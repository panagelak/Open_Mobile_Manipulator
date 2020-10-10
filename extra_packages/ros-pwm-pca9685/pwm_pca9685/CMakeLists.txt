cmake_minimum_required(VERSION 2.8.3)
project(pwm_pca9685)

add_compile_options(-std=c++11 -li2c)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs)

catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pca9685_node src/pca9685_activity.cpp src/pca9685_node.cpp)

include_directories(${catkin_INCLUDE_DIRS})

target_link_libraries(pca9685_node ${catkin_LIBRARIES})

install(TARGETS pca9685_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

