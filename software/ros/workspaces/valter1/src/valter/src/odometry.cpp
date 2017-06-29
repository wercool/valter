//wheelRadius=135mm
//ticksPerRevolution=60
//distanceBetweenWheels=385

#include<iostream>    //cout
#include<stdio.h> //printf
#include<string.h>    //strlen
#include<string>  //string
#include<sys/socket.h>    //socket
#include<arpa/inet.h> //inet_addr
#include<netdb.h> //hostent
#include<math.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

using namespace std;

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

static std::vector<std::string> split(const std::string &text, char sep)
{
    std::vector<std::string> tokens;
    std::size_t start = 0, end = 0;
    while ((end = text.find(sep, start)) != std::string::npos)
    {
        tokens.push_back(text.substr(start, end - start));
        start = end + 1;
    }
    tokens.push_back(text.substr(start));
    return tokens;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "valter_encoders_odometry_publisher");

    ros::NodeHandle n;
    ros::Publisher valter_odom_pub = n.advertise<nav_msgs::Odometry>("encoders_odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    double x  = 0.0;
    double y  = 0.0;
    double th = 0.0;

    double vx  = 0.0;
    double vy  = 0.0;
    double vth = 0.0;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time    = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(5.0);

    double distancePerCount = (2 * M_PI * 0.135) / 60; // (2*PI*wheelRadius)/ticksPerRevolution
    double lengthBetweenTwoWheels = 0.385;

    int prevLeftWheelTicks  = 0;
    int prevRightWheelTicks = 0;

    while(n.ok())
    {
        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();

        int leftWheelTicks  = 0;
        int rightWheelTicks = 0;

        double deltaLeft  = 0.0;
        double deltaRight = 0.0;

        double vLeft  = 0.0;
        double vRight = 0.0;

        int sock = socket(AF_INET , SOCK_STREAM , 0);
        struct sockaddr_in server;
        server.sin_addr.s_addr = inet_addr("192.168.101.101");
        server.sin_family = AF_INET;
        server.sin_port = htons(55555);

        //Connect to remote server
        if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
        {
            perror("Connect failed. Error");
        }
        else
        {
            int returnedBytes;
            std::string request = "ENCODERS";
            char buffer[1024];

            for (int i = 0; i < 1024; i++)
            {
                buffer[i] = '\0';
            }

            returnedBytes = send(sock , request.c_str() , strlen(request.c_str()) , 0);

            if(returnedBytes < 0)
            {
                perror("Send failed. Error");
            }
            else
            {
                int resultBytes = recv(sock, buffer, 1024, 0);
                std::string resultBuffer(buffer);
                std::vector<std::string> resultBufferElements = split(resultBuffer, ',');

                ROS_INFO("L=%s, R=%s", resultBufferElements[0].c_str(), resultBufferElements[1].c_str());

                leftWheelTicks  = atoi(resultBufferElements[0].c_str());
                rightWheelTicks = atoi(resultBufferElements[1].c_str());
            }
        }
        close(sock);

        //extract wheel velocities from the tick signals count
        deltaLeft  = (abs(leftWheelTicks)  - abs(prevLeftWheelTicks))  * (leftWheelTicks  > 0 ? 1 : -1);
        deltaRight = (abs(rightWheelTicks) - abs(prevRightWheelTicks)) * (rightWheelTicks > 0 ? 1 : -1);

        vLeft  = (deltaLeft  * distancePerCount) / (current_time - last_time).toSec();
        vRight = (deltaRight * distancePerCount) / (current_time - last_time).toSec();

        vx  = ((vRight + vLeft) / 2) * 1;
        vy  = 0;
        vth = ((vRight - vLeft) / lengthBetweenTwoWheels) * 1;

        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th)) * dt;
        double delta_y = (vx * sin(th)) * dt;
        double delta_th = vth * dt;

        x  += delta_x;
        y  += delta_y;
        th += delta_th;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "encoders_odom";
        odom_trans.child_frame_id = "Base";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //Odometry message
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "encoders_odom";

        //set the position
        odom.pose.pose.position.x  = x;
        odom.pose.pose.position.y  = y;
        odom.pose.pose.position.z  = 0.0;
        odom.pose.pose.orientation = odom_quat;

         //set the velocity
        odom.child_frame_id = "Base";
        odom.twist.twist.linear.x  = vx;
        odom.twist.twist.linear.y  = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        valter_odom_pub.publish(odom);

        last_time = current_time;
        prevLeftWheelTicks  = leftWheelTicks;
        prevRightWheelTicks = rightWheelTicks;

        r.sleep();
    }
}

