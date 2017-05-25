#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

#include<iostream>    //cout
#include<stdio.h> //printf
#include<string.h>    //strlen
#include<string>  //string
#include<sys/socket.h>    //socket
#include<arpa/inet.h> //inet_addr
#include<netdb.h> //hostent
#include <math.h>

//https://github.com/tue-robotics/tue_manipulation/blob/master/src/torso_server.cpp

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

static std::string& implode(const std::vector<std::string>& elems, char delim, std::string& s)
{
    for (std::vector<std::string>::const_iterator ii = elems.begin(); ii != elems.end(); ++ii)
    {
        s += (*ii);
        if ( ii + 1 != elems.end() )
        {
            s += delim;
        }
    }
    return s;
}

class LeftArmTrajectoryFollower
{
protected:

  ros::NodeHandle nh;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as; 
  std::string action_name;

public:

  LeftArmTrajectoryFollower(std::string name) :
    as(nh, name, false),
    action_name(name)
  {
    //Register callback functions:
    as.registerGoalCallback(boost::bind(&LeftArmTrajectoryFollower::jointTrajectoryAnglesCB, this));
    as.registerPreemptCallback(boost::bind(&LeftArmTrajectoryFollower::preemptCB, this));

    as.start();
  }

  ~LeftArmTrajectoryFollower(void)//Destructor
  {
  }

  void jointTrajectoryAnglesCB()
  {
    control_msgs::FollowJointTrajectoryGoalConstPtr goal = as.acceptNewGoal();
    std::vector<trajectory_msgs::JointTrajectoryPoint> pointVector(goal->trajectory.points);
    ROS_INFO("%s THE SIZE OF THE TRAJECTORY IS %d POINTS",action_name.c_str(), (int) pointVector.size());


    std::vector<std::string> joint_names = goal->trajectory.joint_names;

    std::string joints;
    implode(joint_names, ',', joints);
    ROS_INFO("JOINTS [%s]", joints.c_str());

    //send ACL (Arm Control Left) tasks from MoveIt! [left_arm] group
    int sock = socket(AF_INET , SOCK_STREAM , 0);
    struct sockaddr_in server;
    server.sin_addr.s_addr = inet_addr("192.168.101.101");
    server.sin_family = AF_INET;
    server.sin_port = htons(55555);

    int LTorsoJoint_positions_idx = 0;
    int LShoulderJoint_positions_idx = 0;
    int LArmJoint_positions_idx = 0;
    int LArmElbowJoint_positions_idx = 0;
    int LForearmRollJoint_positions_idx = 0;

    vector<string>::iterator iter = joint_names.begin();
    int idx = 0;
    while( iter != joint_names.end() )
    {
        string jointName = *iter++;
        if (jointName == "LTorsoJoint")
        {
            LTorsoJoint_positions_idx = idx;
        }
        if (jointName == "LShoulderJoint")
        {
            LShoulderJoint_positions_idx = idx;
        }
        if (jointName == "LArmJoint")
        {
            LArmJoint_positions_idx = idx;
        }
        if (jointName == "LArmElbowJoint")
        {
            LArmElbowJoint_positions_idx = idx;
        }
        if (jointName == "LForearmRollJoint")
        {
            LForearmRollJoint_positions_idx = idx;
        }
        idx++;
    }

    int pointVector_pos = pointVector.size() - 1;

    double LArmElbowJointPositionDeg = goal->trajectory.points[pointVector_pos].positions[LArmElbowJoint_positions_idx] * 180 / M_PI;
    double LArmJointPositionDeg = goal->trajectory.points[pointVector_pos].positions[LArmJoint_positions_idx] * 180 / M_PI;
    double LShoulderJointPositionDeg = goal->trajectory.points[pointVector_pos].positions[LShoulderJoint_positions_idx] * 180 / M_PI;
    double LTorsoJointPositionDeg = goal->trajectory.points[pointVector_pos].positions[LTorsoJoint_positions_idx] * 180 / M_PI;
    double LForearmRollJointPositionDeg = abs(goal->trajectory.points[pointVector_pos].positions[LForearmRollJoint_positions_idx]) * 180 / M_PI;

    //Connect to remote server
    if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
    {
        perror("connect failed. Error");
    }
    else
    {
        int returnedBytes;

        std::string request = format("T_ACL_SetLeftLimbPositionTask_%.2f\nT_ACL_SetLeftArmPositionTask_%.2f\nT_ACL_SetLeftForearmPositionTask_%.2f\nT_BCP1_SetLeftArmYawPositionTask_%.2f\nT_ACL_SetLeftArmRollPositionTask_%.2f", 
                                     LShoulderJointPositionDeg,
                                     LArmJointPositionDeg,
                                     LArmElbowJointPositionDeg,
                                     LTorsoJointPositionDeg,
                                     LForearmRollJointPositionDeg);

        char buffer[1024];

        for (int i = 0; i < 1024; i++)
        {
            buffer[i] = '\0';
        }

        returnedBytes = send(sock , request.c_str() , strlen(request.c_str()) , 0);

        ROS_INFO("Tasks script is  sent:\n==============================\n%s\n==============================\n", request.c_str());

        if(returnedBytes < 0)
        {
            perror("Send failed : ");
        }
        else
        {
            int resultBytes = recv(sock, buffer, 1024, 0);
            std::string resultBuffer(buffer);

            for (int i = 0; i < 1024; i++)
            {
                buffer[i] = '\0';
            }
        }
        close(sock);
    }

    control_msgs::FollowJointTrajectoryResult result;

    double goalToleranceDeg = 5.0;
    bool goalReached = false;

    int waitSec = 0;
    while (waitSec < 30)// wait for goal achieved for 30 sec
    {
        int sock = socket(AF_INET , SOCK_STREAM , 0);
        struct sockaddr_in server;
        server.sin_addr.s_addr = inet_addr("192.168.101.101");
        server.sin_family = AF_INET;
        server.sin_port = htons(55555);

        bool LArmElbowJointCompleted = false;
        bool LArmJointCompleted = false;
        bool LShoulderJointCompleted = false;
        bool LTorsoJointCompleted = false;
        bool LForearmRollJointCompleted = false;

        double LArmElbowJointCurrentPositionDeg = 0.0;
        double LArmJointCurrentPositionDeg = 0.0;
        double LShoulderJointCurrentPositionDeg = 0.0;
        double LTorsoJointCurrentPositionDeg = 0.0;
        double LForearmRollJointCurrentPositionDeg = 0.0;

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

                ROS_INFO("%s, %s, %s, %s, %s, %s, %s, %s, %s, %s", resultBufferElements[0].c_str(), resultBufferElements[1].c_str(), resultBufferElements[2].c_str(), resultBufferElements[3].c_str(), resultBufferElements[4].c_str(),
                                                                   resultBufferElements[5].c_str(), resultBufferElements[6].c_str(), resultBufferElements[7].c_str(), resultBufferElements[8].c_str(), resultBufferElements[9].c_str());

                LArmElbowJointCurrentPositionDeg = atof(resultBufferElements[0].c_str());
                LArmJointCurrentPositionDeg = atof(resultBufferElements[1].c_str());
                LShoulderJointCurrentPositionDeg = atof(resultBufferElements[2].c_str());
                LTorsoJointCurrentPositionDeg = atof(resultBufferElements[3].c_str());
                LForearmRollJointCurrentPositionDeg = atof(resultBufferElements[4].c_str());

                double LArmElbowJointPositionRad = LArmElbowJointCurrentPositionDeg * M_PI / 180;
                double LArmJointPositionRad = LArmJointCurrentPositionDeg * M_PI / 180;
                double LShoulderJointPositionRad = LShoulderJointCurrentPositionDeg * M_PI / 180;
                double LTorsoJointPositionRad = LTorsoJointCurrentPositionDeg * M_PI / 180;
                double LForearmRollJointPositionRad = LForearmRollJointCurrentPositionDeg * M_PI / 180;

                ROS_INFO("LArmElbowJoint: %.2f, LArmJoint:%.2f, LShoulderJoint:%.2f, LTorsoJoint:%.2f, LForearmRollJoint:%.2f", 
                          LArmElbowJointCurrentPositionDeg, 
                          LArmJointCurrentPositionDeg, 
                          LShoulderJointCurrentPositionDeg, 
                          LTorsoJointCurrentPositionDeg, 
                          LForearmRollJointCurrentPositionDeg);

/*
                std::vector<double> actual_joint_positions;

                actual_joint_positions.push_back(LArmElbowJointPositionRad);
                actual_joint_positions.push_back(LArmJointPositionRad);
                actual_joint_positions.push_back(0.0);
                actual_joint_positions.push_back(LShoulderJointPositionRad);
                actual_joint_positions.push_back(LTorsoJointPositionRad);

                trajectory_msgs::JointTrajectoryPoint actualTrajectoryPoint;
                actualTrajectoryPoint.positions = actual_joint_positions;

                control_msgs::FollowJointTrajectoryFeedback feedback;
                // Publish feedback
                feedback.header.stamp = ros::Time::now();
                feedback.joint_names = goal->trajectory.joint_names;
                feedback.actual = actualTrajectoryPoint;
                feedback.desired.positions = goal->trajectory.points[pointVector_pos].positions;
                // ToDo: error?
                as.publishFeedback(feedback);
*/
                for (int i = 0; i < 1024; i++)
                {
                    buffer[i] = '\0';
                }
            }
            close(sock);
        }

        LArmElbowJointCompleted = (abs(LArmElbowJointCurrentPositionDeg - LArmElbowJointPositionDeg) <= goalToleranceDeg) ? true : false;
        LArmJointCompleted = (abs(LArmJointCurrentPositionDeg - LArmJointPositionDeg) <= goalToleranceDeg) ? true : false;
        LShoulderJointCompleted = (abs(LShoulderJointCurrentPositionDeg - LShoulderJointPositionDeg) <= goalToleranceDeg) ? true : false;
        LTorsoJointCompleted = (abs(LTorsoJointCurrentPositionDeg - LTorsoJointPositionDeg) <= goalToleranceDeg) ? true : false;
        LForearmRollJointCompleted = (abs(LForearmRollJointCurrentPositionDeg - LForearmRollJointPositionDeg) <= goalToleranceDeg) ? true : false;

        if (LArmElbowJointCompleted && LArmJointCompleted && LShoulderJointCompleted && LTorsoJointCompleted && LForearmRollJointCompleted)
        {
            ROS_INFO("Goal reached in %d sec", waitSec);
            as.setSucceeded(result);
            goalReached = true;
            break;
        }
        waitSec++;
        ROS_INFO("Achieving the goal, waiting %d sec", waitSec);
        ros::Duration(1.0).sleep();
    }

    if (!goalReached)
    {
        ROS_INFO("Reach goal attempt failed after %d sec of waiting", waitSec);
        as.setAborted(result);
    }
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name.c_str());
    // set the action state to preempted
    as.setPreempted();
  }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "left_arm_action_server");
    ROS_INFO("left_arm_action_server node started");

    LeftArmTrajectoryFollower LeftArmTrajectoryFollower("/left_arm_controller/joint_trajectory_action");
    ros::spin();

    return 0;
}
