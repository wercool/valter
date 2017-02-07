#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>

#include<iostream>    //cout
#include<stdio.h> //printf
#include<string.h>    //strlen
#include<string>  //string
#include<sys/socket.h>    //socket
#include<arpa/inet.h> //inet_addr
#include<netdb.h> //hostent
#include <signal.h>
#include <stdlib.h>

#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

typedef void (*sig_t)(int);

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

void exitHandler(int s)
{
    ROS_INFO("Caught signal %d\n",s);
    exit(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "valter_android_teleop_cmd_vel_pub");

    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    geometry_msgs::Twist cmd_vel;

    signal (SIGINT, exitHandler);

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Rate r(5);

    ROS_INFO("Bluetooth Server");

    struct sockaddr_rc loc_addr = { 0 }, rem_addr = { 0 };
    char buf[1024] = { 0 };
    int s, client, bytes_read;
    socklen_t opt = sizeof(rem_addr);

    // allocate socket
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // bind socket to port 1 of the first available 
    // local bluetooth adapter
    loc_addr.rc_family = AF_BLUETOOTH;

    char address[18] = "B8:27:EB:B1:FD:6D";

//    loc_addr.rc_bdaddr = *BDADDR_ANY;
    str2ba( address, &loc_addr.rc_bdaddr);
    loc_addr.rc_channel = (uint8_t) 1;
    bind(s, (struct sockaddr *)&loc_addr, sizeof(loc_addr));

    // put socket into listening mode
    listen(s, 1);

    // accept one connection
    client = accept(s, (struct sockaddr *)&rem_addr, &opt);

    ba2str( &rem_addr.rc_bdaddr, buf );
    ROS_INFO("accepted connection from %s", buf);
    memset(buf, 0, sizeof(buf));

    // read data from the client

    while (ros::ok())
    {
       bytes_read = read(client, buf, sizeof(buf));
       buf[bytes_read] = '\0';
       if( bytes_read > 0 )
       {
           ROS_INFO("received [%s] %d bytes", buf, bytes_read);

           char* cmd_vel_parts = strtok(buf, ",");
           
           float linVel = (float) atof(cmd_vel_parts);
           
           cmd_vel_parts = strtok(NULL, ",");

           float angVel = (float) atof(cmd_vel_parts);

           ROS_INFO("linVel: %.3f, angVel: %.3f", linVel, angVel);

           cmd_vel.linear.x = linVel;
           cmd_vel.linear.y = 0;
           cmd_vel.angular.z = angVel;

           cmd_vel_pub.publish(cmd_vel);
       }
       else
       {
           ROS_INFO("connection closed");
           break;
       }
       ros::spinOnce();
       r.sleep();
    }

    // close connection
    close(client);
    close(s);
    return 0;
}
