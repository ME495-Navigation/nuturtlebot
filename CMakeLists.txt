cmake_minimum_required(VERSION 3.0.9)
project(nuturtlebot)

find_package(catkin REQUIRED COMPONENTS )

generate_messages(
  DEPENDENCIES
)

catkin_package()

install(PROGRAMS  scripts/build_firmware
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )

 #build_firmware expects arduino-cli.yaml to be in the same directory so
 # install it to the bin directory
install(FILES scripts/arduino-cli.yaml
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
