#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ros/console.h>

#include<iostream>    //cout
#include<stdio.h> //printf
#include<string.h>    //strlen
#include<string>  //string
#include<sys/socket.h>    //socket
#include<arpa/inet.h> //inet_addr
#include<netdb.h> //hostent

#include <string>
#include <cstdarg>

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

class ValterJoyTeleop
{
public:
  ValterJoyTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int linear_, angular_;
  float prevLin, prevAng;
  ros::Subscriber joy_sub_;
  
};

ValterJoyTeleop::ValterJoyTeleop()
{
  prevLin = prevAng = -1;
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &ValterJoyTeleop::joyCallback, this);
}


void ValterJoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  float linVel = joy->axes[1];
  float angVel = joy->axes[0];

  linVel = linVel * 0.12;
  angVel = angVel * 0.587;

  if (fabs(prevLin - linVel) > 0.01 || fabs(prevAng - angVel) > 0.01)
  {

    //ROS_INFO("linVel = %f, angVel = %f", linVel, angVel);

    std::string cmdVelTaskScriptLine = format("T_PCP1_CmdVelTask_%.2f_%.2f", linVel, angVel);

    ROS_INFO("%s", cmdVelTaskScriptLine.c_str());

    int sock = socket(AF_INET , SOCK_STREAM , 0);
    struct sockaddr_in server;
    server.sin_addr.s_addr = inet_addr( "192.168.0.247" );
    server.sin_family = AF_INET;
    server.sin_port = htons( 55555 );

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

    prevLin = linVel;
    prevAng = angVel;
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "valter_joy_teleop");
  ValterJoyTeleop valter_joy_teleop;

  ros::spin();
}
