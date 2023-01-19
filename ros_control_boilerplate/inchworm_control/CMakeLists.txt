include_directories(
  include/
)

# Inchworm Hardware Interface
add_library(inchworm_hw_interface
  src/inchworm_hw_interface.cpp
)
target_link_libraries(inchworm_hw_interface
  generic_hw_interface
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(inchworm_hw_main src/inchworm_hw_main.cpp)
target_link_libraries(inchworm_hw_main
  inchworm_hw_interface
  generic_hw_control_loop
  ${catkin_LIBRARIES}
)

## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    inchworm_hw_interface
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    inchworm_hw_main
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/inchworm_control/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
