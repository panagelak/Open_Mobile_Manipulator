cmake_minimum_required(VERSION 2.8.3)
project(spin_action_server)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  message_runtime
  rospy
)



## Generate actions in the 'action' folder
add_action_files(
   FILES
   Spin.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs
 )


catkin_package(
# INCLUDE_DIRS include
  LIBRARIES spin_action_server
  CATKIN_DEPENDS actionlib actionlib_msgs message_generation message_runtime rospy
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)