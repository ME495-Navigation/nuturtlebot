cmake_minimum_required(VERSION 3.0.9)
project(nuturtlebot)

find_package(catkin REQUIRED COMPONENTS )

catkin_package()

add_custom_target(raspi_image ALL COMMAND ${CMAKE_CURRENT_LIST_DIR}/scripts/create_image)

install(PROGRAMS  scripts/build_firmware scripts/create_image scripts/write_image
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )

 #build_firmware expects arduino-cli.yaml to be in the same directory so
 # install it to the bin directory
install(FILES scripts/arduino-cli.yaml
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
