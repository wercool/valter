cmake_minimum_required(VERSION 2.8.3)
project(valter)


set(OpenCV_DIR /home/maska/opencv-2.4.13/release/)
find_package(OpenCV REQUIRED)

## Find catkin and any catkin packages
find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy tf std_msgs sensor_msgs nav_msgs move_base_msgs geometry_msgs moveit_msgs image_transport cv_bridge) #moveit_ros_planning_interface)
find_package(Eigen REQUIRED)

## Declare a catkin package
catkin_package(
)

include_directories( ${catkin_INCLUDE_DIRS} )
include_directories(${Eigen_INCLUDE_DIRS})

add_executable(valter_joy_teleop src/valter_joy_teleop.cpp)
target_link_libraries(valter_joy_teleop ${catkin_LIBRARIES})

add_executable(valter_joy_teleop_cmd_vel_pub src/valter_joy_teleop_cmd_vel_pub.cpp)
target_link_libraries(valter_joy_teleop_cmd_vel_pub ${catkin_LIBRARIES})

add_executable(face_tracker src/face_tracker.cpp)
target_link_libraries(face_tracker ${catkin_LIBRARIES})
