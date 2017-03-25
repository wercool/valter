#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <ros/console.h>

#include<iostream>    //cout
#include<stdio.h> //printf
#include<string.h>    //strlen
#include<string>  //string
#include<sys/socket.h>    //socket
#include<arpa/inet.h> //inet_addr
#include<netdb.h> //hostent
#include <math.h>

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
  ros::init(argc, argv, "valter_joint_states_publisher");

  ros::NodeHandle nh;

  ros::Publisher left_arm_joint_states_publisher = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

  ros::Rate r(10.0);
  while(nh.ok())
  {

    ros::spinOnce();               // check for incoming messages

    //MOVEITLEFTARMGROUP left_arm group
    int sock = socket(AF_INET , SOCK_STREAM , 0);
    struct sockaddr_in server;
    server.sin_addr.s_addr = inet_addr("192.168.101.101");
    server.sin_family = AF_INET;
    server.sin_port = htons(55555);

    //Connect to remote server
    if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
    {
        perror("connect failed. Error");
    }
    else
    {
        int returnedBytes;
        std::string request = "MOVEITJOINTSSTATE";
        char buffer[1024];

        for (int i = 0; i < 1024; i++)
        {
            buffer[i] = '\0';
        }

        returnedBytes = send(sock , request.c_str() , strlen(request.c_str()) , 0);

        if(returnedBytes < 0)
        {
            perror("Send failed : ");
        }
        else
        {
            int resultBytes = recv(sock, buffer, 1024, 0);
            std::string resultBuffer(buffer);
            std::vector<std::string> resultBufferElements = split(resultBuffer, ',');

            ROS_INFO("%s, %s, %s, %s, %s, %s, %s, %s", resultBufferElements[0].c_str(), resultBufferElements[1].c_str(), resultBufferElements[2].c_str(), resultBufferElements[3].c_str(),
                                                       resultBufferElements[4].c_str(), resultBufferElements[5].c_str(), resultBufferElements[6].c_str(), resultBufferElements[7].c_str());

            double LArmElbowJointPositionRad = atof(resultBufferElements[0].c_str()) * M_PI / 180;
            double LArmJointPositionRad = atof(resultBufferElements[1].c_str()) * M_PI / 180;
            double LShoulderJointPositionRad = atof(resultBufferElements[2].c_str()) * M_PI / 180;
            double LTorsoJointPositionRad = atof(resultBufferElements[3].c_str()) * M_PI / 180;

            double RArmElbowJointPositionRad = atof(resultBufferElements[4].c_str()) * M_PI / 180;
            double RArmJointPositionRad = atof(resultBufferElements[5].c_str()) * M_PI / 180;
            double RShoulderJointPositionRad = atof(resultBufferElements[6].c_str()) * M_PI / 180;
            double RTorsoJointPositionRad = atof(resultBufferElements[7].c_str()) * M_PI / 180;

            ROS_INFO("LArmElbowJoint: %.2f, LArmJoint:%.2f, LShoulderJoint:%.2f, LTorsoJoint:%.2f, RArmElbowJoint: %.2f, RArmJoint:%.2f, RShoulderJoint:%.2f, RTorsoJoint:%.2f", 
                      LArmElbowJointPositionRad, LArmJointPositionRad, LShoulderJointPositionRad, LTorsoJointPositionRad,
                      RArmElbowJointPositionRad, RArmJointPositionRad, RShoulderJointPositionRad, RTorsoJointPositionRad);


            std::vector<std::string> joint_names;
            std::vector<double> joint_positions;
            std::vector<double> joint_velocities;
            std::vector<double> joint_efforts;

            joint_names.push_back("LArmElbowJoint");
            joint_positions.push_back(LArmElbowJointPositionRad);
            joint_velocities.push_back(0.0);
            joint_efforts.push_back(0.0);

            joint_names.push_back("LArmJoint");
            joint_positions.push_back(LArmJointPositionRad);
            joint_velocities.push_back(0.0);
            joint_efforts.push_back(0.0);

            joint_names.push_back("LShoulderJoint");
            joint_positions.push_back(LShoulderJointPositionRad);
            joint_velocities.push_back(0.0);
            joint_efforts.push_back(0.0);

            joint_names.push_back("LTorsoJoint");
            joint_positions.push_back(LTorsoJointPositionRad);
            joint_velocities.push_back(0.0);
            joint_efforts.push_back(0.0);

            joint_names.push_back("RArmElbowJoint");
            joint_positions.push_back(RArmElbowJointPositionRad);
            joint_velocities.push_back(0.0);
            joint_efforts.push_back(0.0);

            joint_names.push_back("RArmJoint");
            joint_positions.push_back(RArmJointPositionRad);
            joint_velocities.push_back(0.0);
            joint_efforts.push_back(0.0);

            joint_names.push_back("RShoulderJoint");
            joint_positions.push_back(RShoulderJointPositionRad);
            joint_velocities.push_back(0.0);
            joint_efforts.push_back(0.0);

            joint_names.push_back("RTorsoJoint");
            joint_positions.push_back(RTorsoJointPositionRad);
            joint_velocities.push_back(0.0);
            joint_efforts.push_back(0.0);

            sensor_msgs::JointState left_arm_joint_states;
            left_arm_joint_states.header.stamp = ros::Time::now();
            left_arm_joint_states.name = joint_names;
            left_arm_joint_states.position = joint_positions;
            left_arm_joint_states.velocity = joint_velocities;
            left_arm_joint_states.effort = joint_efforts;
            left_arm_joint_states_publisher.publish(left_arm_joint_states);

            for (int i = 0; i < 1024; i++)
            {
                buffer[i] = '\0';
            }
        }
        close(sock);
    }

    r.sleep();
  }
}
