include_directories(
  include/
)

# Ommp Hardware Interface
add_library(ommp_hw_interface
  src/ommp_hw_interface.cpp
)
target_link_libraries(ommp_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(ommp_hw_main src/ommp_hw_main.cpp)
target_link_libraries(ommp_hw_main
  ommp_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    ommp_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    ommp_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/ommp_control/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
