cmake_minimum_required(VERSION 3.0.2)
project(ommp_moveit_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometric_shapes
  message_generation
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  pluginlib
  roscpp
  rospy
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED COMPONENTS system)


## Generate services in the 'srv' folder
add_service_files(
  FILES
  PickPlace.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs
 geometry_msgs
)

catkin_package(
)


include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})


add_executable(pick_place_server src/pick_place_server.cpp)
add_dependencies(pick_place_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(pick_place_server ${catkin_LIBRARIES})

