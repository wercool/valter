cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg serial roslib sensor_msgs)

#set(serial_SRCS src/serial/src/serial.cc src/serial/include/serial/serial.h src/serial/include/serial/v8stdint.h)
#list(APPEND serial_SRCS src/serial/src/impl/unix.cc)
#add_library(serial ${serial_SRCS})
#target_link_libraries(serial rt)


find_package(Qt4 4.6 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})


## Declare ROS messages and services
add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
)


set(turtlesim_node_SRCS
src/listener.cpp
)

set(turtlesim_node_HDRS
include/turtle_frame.h
)

qt4_wrap_cpp(turtlesim_node_MOCS ${turtlesim_node_HDRS})


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
#add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener ${turtlesim_node_SRCS} ${turtlesim_node_MOCS})
target_link_libraries(listener ${QT_LIBRARIES} ${catkin_LIBRARIES})
#add_dependencies(listener beginner_tutorials_generate_messages_cpp)

add_executable(joy_listener src/joy_listener.cpp ${turtlesim_node_MOCS})
target_link_libraries(joy_listener ${QT_LIBRARIES} ${catkin_LIBRARIES})


add_dependencies(talker ${catkin_EXPORTED_TARGETS}
add_dependencies(listener ${catkin_EXPORTED_TARGETS}))
