cmake_minimum_required(VERSION 2.8.3)
project(ommp_demo)

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

find_package(catkin
  REQUIRED COMPONENTS
  roscpp
  roslib
  actionlib
  integration
  gazebo_ros_link_attacher
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES station_handlers
  CATKIN_DEPENDS roscpp roslib actionlib integration gazebo_ros_link_attacher
#  DEPENDS boost
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
# ${Boost_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(ommp_demo src/ommp_demo.cpp)
target_link_libraries(ommp_demo   ${catkin_LIBRARIES})
add_dependencies(ommp_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



