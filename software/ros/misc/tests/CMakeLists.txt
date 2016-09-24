cmake_minimum_required(VERSION 2.8.3)
project(valter)

## Find catkin and any catkin packages
#find_package(catkin REQUIRED COMPONENTS roscpp rospy tf std_msgs sensor_msgs nav_msgs move_base_msgs actionlib)
find_package(catkin REQUIRED COMPONENTS roscpp rospy tf std_msgs sensor_msgs nav_msgs move_base_msgs geometry_msgs)

#include_directories(include ${catkin_INCLUDE_DIRS})
#add_executable(state_publisher src/state_publisher.cpp)
#target_link_libraries(state_publisher ${catkin_LIBRARIES})

#add_executable(odometry src/odometry.cpp)
#target_link_libraries(odometry ${catkin_LIBRARIES})

#add_executable(simple_navigation_goals src/simple_navigation_goals.cpp)
#target_link_libraries(simple_navigation_goals ${catkin_LIBRARIES})

#add_executable(tf_broadcaster src/tf_broadcaster.cpp)
#target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

add_executable(move_base_simple src/move_base_simple.cpp)
target_link_libraries(move_base_simple ${catkin_LIBRARIES})

add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES})
