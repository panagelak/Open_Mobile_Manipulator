cmake_minimum_required(VERSION 2.8.3)
project(moveit_jog_arm)

add_compile_options(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  geometry_msgs
  #rain_unity
  moveit_ros_planning_interface
  rosparam_shortcuts
  tf2_ros
)
find_package(Eigen3 REQUIRED)


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    control_msgs
    geometry_msgs
    #rain_unity
    moveit_ros_planning_interface
    rosparam_shortcuts
    tf2_ros
  DEPENDS
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(jog_server
  src/collision_check_thread.cpp
  src/jog_server.cpp
  src/jog_calcs.cpp
  src/jog_ros_interface.cpp
  src/low_pass_filter.cpp
)

add_dependencies(jog_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(jog_server ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(spacenav_to_twist
  src/teleop_examples/spacenav_to_twist.cpp
)
add_dependencies(spacenav_to_twist ${catkin_EXPORTED_TARGETS})
target_link_libraries(spacenav_to_twist ${catkin_LIBRARIES})

add_executable(xbox_to_twist
  src/teleop_examples/xbox_to_twist.cpp
)
add_dependencies(xbox_to_twist ${catkin_EXPORTED_TARGETS})
target_link_libraries(xbox_to_twist ${catkin_LIBRARIES})

install(
  TARGETS
    jog_server
    spacenav_to_twist
    xbox_to_twist
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

