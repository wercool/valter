#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "MoveGroupInterface", ros::init_options::AnonymousName);

  ros::NodeHandle n;
  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("left_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


/*
    pose: 
  position: 
    x: 0.688820123672
    y: 0.278224915266
    z: 1.1044973135
  orientation: 
    x: 0.674523174763
    y: -0.27303108573
    z: -0.671703577042
    w: -0.138877004385

*/

  geometry_msgs::Pose target_pose;
  target_pose.position.x = 0.688820123672;
  target_pose.position.y = 0.278224915266;
  target_pose.position.z = 1.1044973135;
  target_pose.orientation.x = 0.674523174763;
  target_pose.orientation.y = -0.27303108573;
  target_pose.orientation.z = -0.671703577042;
  target_pose.orientation.w = -0.138877004385;
  group.setPoseTarget(target_pose);



  moveit::planning_interface::MoveGroup::Plan plan;
  bool success = group.plan(plan);
  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  // Sleep to give Rviz time to visualize the plan.
  sleep(5.0);
  group.move();

  ros::waitForShutdown();
}
