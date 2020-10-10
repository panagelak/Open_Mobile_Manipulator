cmake_minimum_required(VERSION 2.8.3)
project(integration)

find_package(catkin
  REQUIRED COMPONENTS
  actionlib_msgs
  std_msgs
  geometry_msgs
  message_generation
  sensor_msgs
  )

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
)

## Generate services in the 'srv' folder
 add_service_files(
  DIRECTORY srv
 )

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
   geometry_msgs
   sensor_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES integration
  CATKIN_DEPENDS actionlib_msgs std_msgs message_runtime geometry_msgs sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

