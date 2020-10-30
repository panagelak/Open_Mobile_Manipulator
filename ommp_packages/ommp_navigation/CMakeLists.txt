cmake_minimum_required(VERSION 2.8.3)
project(ommp_navigation)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)