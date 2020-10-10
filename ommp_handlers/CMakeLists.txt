cmake_minimum_required(VERSION 3.0.2)
project(ommp_handlers)

#####################################
## Check c++11 / c++0x support ######
#####################################
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
  message(FATAL_ERROR "Compiler ${CMAKE_CXX_COMPILER} has no C++11 support.")
endif()
#####################################

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  sensor_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  rospy
  integration
  gazebo_ros_link_attacher
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED COMPONENTS system)



catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES station_handlers
  CATKIN_DEPENDS 
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  )

install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS
        PATTERN ".svn" EXCLUDE)

## Declare a C++ executable for Movejoints

add_executable(moveJoints_handler_server_node src/moveJoints_handler_server_node.cpp)
target_link_libraries(moveJoints_handler_server_node   ${catkin_LIBRARIES})
add_dependencies(moveJoints_handler_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(moveCartesian_arm_handler_server_node src/moveCartesian_arm_handler_server_node.cpp)
target_link_libraries(moveCartesian_arm_handler_server_node   ${catkin_LIBRARIES})
add_dependencies(moveCartesian_arm_handler_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(attachObject_handler_server_node src/attachObject_handler_server_node.cpp)
target_link_libraries(attachObject_handler_server_node   ${catkin_LIBRARIES})
add_dependencies(attachObject_handler_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(get_frame_distance_server_node src/get_frame_distance_server_node.cpp)
target_link_libraries(get_frame_distance_server_node ${catkin_LIBRARIES})
add_dependencies(get_frame_distance_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(add_planning_box_server_node src/add_planning_box_server_node.cpp)
target_link_libraries(add_planning_box_server_node ${catkin_LIBRARIES})
add_dependencies(add_planning_box_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



