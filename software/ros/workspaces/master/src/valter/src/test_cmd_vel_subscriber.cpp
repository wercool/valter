#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>

#include<iostream>    //cout
#include<stdio.h> //printf
#include<string.h>    //strlen
#include<string>  //string
#include<sys/socket.h>    //socket
#include<arpa/inet.h> //inet_addr
#include<netdb.h> //hostent

//missing string printf
//this is safe and convenient but not exactly efficient
inline std::string format(const char* fmt, ...)
{
    int size = 512;
    char* buffer = 0;
    buffer = new char[size];
    va_list vl;
    va_start(vl, fmt);
    int nsize = vsnprintf(buffer, size, fmt, vl);
    if(size<=nsize){ //fail delete buffer and try again
        delete[] buffer;
        buffer = 0;
        buffer = new char[nsize+1]; //+1 for /0
        nsize = vsnprintf(buffer, size, fmt, vl);
    }
    std::string ret(buffer);
    va_end(vl);
    delete[] buffer;
    return ret;
}

geometry_msgs::Twist cmd_vel;

void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_vel = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_vel_subscriber");

  ros::NodeHandle n;
  ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 100, CmdVelCallback);

  float prevLin, prevAng;
  ros::Time prevSentTime;

  ros::Rate r(30.0);
  while(n.ok())
  {

    ros::spinOnce();               // check for incoming messages

    if (fabs(prevLin - cmd_vel.linear.x) > 0.001 || fabs(prevAng - cmd_vel.angular.z) > 0.001)
    {
      ROS_INFO("linVel = %f, angVel = %f", cmd_vel.linear.x, cmd_vel.angular.z);


      std::string cmdVelTaskScriptLine = format("T_PCP1_CmdVelTask_%.2f_%.2f", cmd_vel.linear.x, cmd_vel.angular.z);

      ROS_INFO("%s", cmdVelTaskScriptLine.c_str());

      int sock = socket(AF_INET , SOCK_STREAM , 0);
      struct sockaddr_in server;
      server.sin_addr.s_addr = inet_addr("192.168.0.100");
      server.sin_family = AF_INET;
      server.sin_port = htons(55555);

      //Connect to remote server
      if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
      {
        perror("connect failed. Error");
      }
      else
      {
        //Send some data
        if( send(sock , cmdVelTaskScriptLine.c_str() , strlen( cmdVelTaskScriptLine.c_str() ) , 0) < 0)
        {
            perror("Send failed : ");
        }
        close(sock);
      }

      prevLin = cmd_vel.linear.x;
      prevAng = cmd_vel.angular.z;
    }

    r.sleep();
  }
}
