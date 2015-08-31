#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

// Robot velocities
geometry_msgs::Twist vel;

geometry_msgs::PoseWithCovarianceStamped initialPose;

double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.00;

bool initialPoseSet = false;

void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    vel = *msg;

    if (initialPoseSet)
    {
        th = tf::getYaw(initialPose.pose.pose.orientation);
        initialPoseSet = false;
    }
}

void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    initialPose = *msg;
    ROS_INFO("Initial Pose [x,y,th]: %f, %f, %f", initialPose.pose.pose.position.x, initialPose.pose.pose.position.y, initialPose.pose.pose.orientation.z);
    initialPoseSet = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odometry", 100);
  ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 100, CmdVelCallback);
  ros::Subscriber pose_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 100, InitialPoseCallback);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(30.0);
  while(n.ok())
  {

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    geometry_msgs::Quaternion odom_quat;

    if (!initialPoseSet)
    {
        vx  = vel.linear.x;
        vy  = vel.linear.y;
        vth = vel.angular.z;

        x  += vx * cos(th);
        y  += vy * sin(th);
        th += vth;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf::createQuaternionMsgFromYaw(th);
    }
    else
    {
        x = initialPose.pose.pose.position.x;
        y = initialPose.pose.pose.position.y;
        odom_quat = initialPose.pose.pose.orientation;
    }

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "Base";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    ROS_INFO("Current Odometry [x,y,th]: %f, %f, %f", x, y, odom.pose.pose.orientation.z);

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
