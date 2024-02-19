cmake_minimum_required(VERSION 2.8.3)
project(manas1)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker_listener_node and talker_listener_node_2
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker_listener_node src/talker_listener_node.cpp)
target_link_libraries(talker_listener_node ${catkin_LIBRARIES})

add_executable(talker_listener_node_2 src/talker_listener_node_2.cpp)
target_link_libraries(talker_listener_node_2 ${catkin_LIBRARIES})
