cmake_minimum_required(VERSION 2.8.3)
project(manas2)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  map_server
  
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs nav_msgs #map_server
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(/home/rohanshenoy30/Downloads/map.yaml)


## Declare a C++ executable

add_executable(Mapping src/Mapping.cpp)
add_executable(MapNodes src/MapNodes.cpp)
add_executable(Astar src/Astar.cpp)


## Specify libraries to link a library or executable target against

target_link_libraries(Mapping ${catkin_LIBRARIES})
target_link_libraries(MapNodes ${catkin_LIBRARIES})
target_link_libraries(Astar ${catkin_LIBRARIES})



target_link_libraries(MapNodes yaml-cpp)  # Adjust this line as needed
