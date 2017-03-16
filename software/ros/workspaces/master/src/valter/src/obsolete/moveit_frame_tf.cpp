#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_frame_tf_publisher");

  ros::NodeHandle n;
  tf::TransformBroadcaster moveit_frame_trans_broadcaster;

  ros::Time current_time;
  geometry_msgs::Quaternion moveit_frame_quat;


  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped moveit_frame_trans;
  moveit_frame_trans.header.frame_id = "moveit_frame_tf";
  moveit_frame_trans.child_frame_id = "odom";

  moveit_frame_trans.transform.translation.x = 0.0;
  moveit_frame_trans.transform.translation.y = 0.0;
  moveit_frame_trans.transform.translation.z = 0.0;
  moveit_frame_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);

  ros::Rate r(30.0);
  while(n.ok())
  {

    current_time = ros::Time::now();

    //first, we'll publish the transform over tf
    moveit_frame_trans.header.stamp = current_time;

    //send the transform
    moveit_frame_trans_broadcaster.sendTransform(moveit_frame_trans);

    r.sleep();
  }
}