cmake_minimum_required(VERSION 2.8.3)
project(valter)


## Find catkin and any catkin packages
find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy tf std_msgs sensor_msgs cv_bridge image_transport)
find_package(Eigen REQUIRED)

## Declare a catkin package
catkin_package(
)

include_directories( ${catkin_INCLUDE_DIRS} )
include_directories(${Eigen_INCLUDE_DIRS})

add_executable(face_tracker src/face_tracker.cpp)
target_link_libraries(face_tracker ${catkin_LIBRARIES})
target_link_libraries(face_tracker ${LINKER_FLAGS})
