cmake_minimum_required(VERSION 2.8.3)
project(valter_movit)

find_package(catkin REQUIRED COMPONENTS roscpp rospy tf std_msgs sensor_msgs moveit_msgs moveit_ros_planning_interface)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(left_arm_joint_trajectory_action_server src/left_arm_joint_trajectory_action_server.cpp)
target_link_libraries(left_arm_joint_trajectory_action_server ${catkin_LIBRARIES})

add_executable(right_arm_joint_trajectory_action_server src/right_arm_joint_trajectory_action_server.cpp)
target_link_libraries(right_arm_joint_trajectory_action_server ${catkin_LIBRARIES})
