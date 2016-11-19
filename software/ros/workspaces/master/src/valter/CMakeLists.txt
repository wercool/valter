cmake_minimum_required(VERSION 2.8.3)
project(valter)


## Find catkin and any catkin packages
find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy tf std_msgs sensor_msgs nav_msgs move_base_msgs geometry_msgs moveit_msgs ) #moveit_ros_planning_interface)
find_package(Eigen REQUIRED)

## Declare a catkin package
catkin_package(
)

include_directories( ${catkin_INCLUDE_DIRS} )
include_directories(${Eigen_INCLUDE_DIRS})

#add_executable(move_base_simple src/move_base_simple.cpp)
#target_link_libraries(move_base_simple ${catkin_LIBRARIES})

#add_executable(odometry src/odometry.cpp)
#target_link_libraries(odometry ${catkin_LIBRARIES})

#add_executable(ir0_scan src/ir0_scan.cpp)
#target_link_libraries(ir0_scan ${catkin_LIBRARIES})

#add_executable(ir1_scan src/ir1_scan.cpp)
#target_link_libraries(ir1_scan ${catkin_LIBRARIES})

#add_executable(point_cloud src/point_cloud.cpp)
#target_link_libraries(point_cloud ${catkin_LIBRARIES})

#add_executable(us0_range src/us0_range.cpp)
#target_link_libraries(us0_range ${catkin_LIBRARIES})

#add_executable(ir0_transform src/ir0_transform.cpp)
#target_link_libraries(ir0_transform ${catkin_LIBRARIES})

#add_executable(set_goal src/set_goal.cpp)
#target_link_libraries(set_goal ${Eigen_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(moveit_frame_tf src/moveit_frame_tf.cpp)
#target_link_libraries(moveit_frame_tf ${catkin_LIBRARIES})

add_executable(valter_joy_teleop src/valter_joy_teleop.cpp)
target_link_libraries(valter_joy_teleop ${catkin_LIBRARIES})

add_executable(valter_joy_teleop_cmd_vel_pub src/valter_joy_teleop_cmd_vel_pub.cpp)
target_link_libraries(valter_joy_teleop_cmd_vel_pub ${catkin_LIBRARIES})

add_executable(test_cmd_vel_subscriber src/test_cmd_vel_subscriber.cpp)
target_link_libraries(test_cmd_vel_subscriber ${catkin_LIBRARIES})
