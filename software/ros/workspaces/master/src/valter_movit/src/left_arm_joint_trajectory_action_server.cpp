#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

//https://github.com/tue-robotics/tue_manipulation/blob/master/src/torso_server.cpp

using namespace std;

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
  ros::Publisher left_arm_joint_states_publisher;
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

    left_arm_joint_states_publisher = nh.advertise<sensor_msgs::JointState>("left_arm_joint_states_publisher", 1);
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
/*
    control_msgs::FollowJointTrajectoryFeedback feedback;

    sensor_msgs::JointState left_arm_joint_states;
    left_arm_joint_states.header.stamp = ros::Time::now();
    left_arm_joint_states.name = goal->trajectory.joint_names;
    left_arm_joint_states.position = goal->trajectory.points[pointVector.size()].positions;
    left_arm_joint_states.velocity = goal->trajectory.points[pointVector.size()].velocities;
    left_arm_joint_states.effort = goal->trajectory.points[pointVector.size()].effort;
    left_arm_joint_states_publisher.publish(left_arm_joint_states);

    // Publish feedback
    feedback.header.stamp = ros::Time::now();
    feedback.actual.positions = goal->trajectory.points[pointVector.size()];
    feedback.desired = goal->trajectory.points[pointVector.size()];
    // ToDo: error?
    as.publishFeedback(feedback);
*/
    control_msgs::FollowJointTrajectoryResult result;
    as.setSucceeded(result);
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
