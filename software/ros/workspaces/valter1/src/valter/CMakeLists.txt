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

add_executable(cmd_vel_subscriber src/cmd_vel_subscriber.cpp)
target_link_libraries(cmd_vel_subscriber ${catkin_LIBRARIES})

add_executable(bluetooth_server src/bluetooth_server.cpp)
target_link_libraries(bluetooth_server ${catkin_LIBRARIES} bluetooth)
