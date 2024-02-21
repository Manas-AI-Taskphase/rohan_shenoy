cmake_minimum_required(VERSION 2.8.3)
project(manas1)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)



## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

## Declare catkin specific configuration to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## INCLUDE_DIRS: you should also specify any other locations where header files may be found

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

## Build talker_listener_node
add_executable(talker_listener_node src/talker_listener_node.cpp)
target_link_libraries(talker_listener_node ${catkin_LIBRARIES})

## Build talker_listener_node_2
add_executable(talker_listener_node_2 src/talker_listener_node_2.cpp)
target_link_libraries(talker_listener_node_2 ${catkin_LIBRARIES})
