cmake_minimum_required(VERSION 2.8.3)
project(kinematic_chains)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  genmsg
  roscpp
  std_msgs
  kdl_parser
  urdf
  tf
  message_generation
  sensor_msgs
  geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS} include/kinematic_chains/)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  CalculateIK.srv
  ChangeEffectorDescription.srv
  InitJoints.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
)

###########
## Build ##
###########
include_directories(
    INCLUDE_DIRS include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(kinematic_chains src/main.cpp src/kinematic_chains.cpp src/manipulator.cpp include/kinematic_chains/manipulator.hpp include/kinematic_chains/kinematic_chains.hpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(kinematic_chains ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(kinematic_chains
  ${catkin_LIBRARIES}
)


