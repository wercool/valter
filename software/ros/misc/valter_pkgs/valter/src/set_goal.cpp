#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

geometry_msgs::Pose target_pose;
bool target_pose_set = false;

// Process incoming planning markers update
void control_goalCb(const visualization_msgs::InteractiveMarkerUpdateConstPtr &update)
{
	if (update->poses.size()!=0)
	{
		ROS_INFO("UPDATE");
	}
}

void eefFeedBack(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	if (feedback->event_type == feedback->MOUSE_UP)
	{
		target_pose = feedback->pose;
		target_pose_set = true;
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "MoveGroupInterface", ros::init_options::AnonymousName);

  ros::NodeHandle n;
/*
  ros::Subscriber interactive_marker_subscriber = n.subscribe<visualization_msgs::InteractiveMarkerUpdate>("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update", 2, control_goalCb);
*/


  ros::Subscriber interactive_marker_feedback_subscriber = n.subscribe<visualization_msgs::InteractiveMarkerFeedback>("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback", 2, eefFeedBack);

  // start a ROS spinning thread
  //ros::AsyncSpinner spinner(1);
  //spinner.start();

  moveit::planning_interface::MoveGroup group("left_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ros::Rate r(10);

  while (ros::ok())
  {
    ROS_INFO("LOOP...");
    if (target_pose_set)
    {
	ROS_INFO("Visualizing plan...");
        group.setPoseTarget(target_pose);
  	ros::AsyncSpinner spinner(1);
  	spinner.start();
  	moveit::planning_interface::MoveGroup::Plan my_plan;
  	bool success = group.plan(my_plan);
  	ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
	target_pose_set = false;
    }
    ros::spinOnce();                   // Handle ROS events
    r.sleep();
  }




/*
  // this connects to a running instance of the move_group node
  move_group_interface::MoveGroup group("left_arm");
  // specify that our target will be a random one
  group.setRandomTarget();
  // plan the motion and then move the group to the sampled target

  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);
//  group.move();
*/

/*




  moveit::planning_interface::MoveGroup group("left_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit::core::RobotState start_state = *group.getCurrentState();
  geometry_msgs::PoseStamped start_pose_stmp= group.getCurrentPose(group.getEndEffector());
  geometry_msgs::Pose start_pose = start_pose_stmp.pose;

  ROS_INFO("Planning Frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("End Effector Link: %s", group.getEndEffectorLink().c_str());
  ROS_INFO("End Effector Link coordinates: x=%f,y=%f,z=%f,w=%f", start_pose.position.x, start_pose.position.y, start_pose.position.z, start_pose.orientation.w);


*/


/*
    pose: 
      position: 
        x: 0.381060705865
        y: 0.16172559261
        z: 0.966743574842
      orientation: 
        x: 0.652408100577
        y: -0.255504472082
        z: -0.636212606247
        w: -0.322977792889

*/
/*
  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = 0.381060705865;
  target_pose1.position.y = 0.16172559261;
  target_pose1.position.z = 0.966743574842;
  target_pose1.orientation.x = 0.652408100577;
  target_pose1.orientation.y = -0.255504472082;
  target_pose1.orientation.z = -0.636212606247;
  target_pose1.orientation.w = -0.322977792889;
  group.setPoseTarget(target_pose1);
*/

/*
  geometry_msgs::Pose target_pose = start_pose;

  start_pose.position.x = 0.142897068949;
  start_pose.position.y = 0.565770382702;
  start_pose.position.z = 0.959782;
  start_pose.orientation.w = 0.476962;

  group.setPoseTarget(start_pose);
*/
/*
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);
  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  // Sleep to give Rviz time to visualize the plan.
  sleep(5.0);
  group.move();
*/
  ros::waitForShutdown();
}
