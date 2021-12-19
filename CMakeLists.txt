cmake_minimum_required(VERSION 3.0.9)
project(nuturtlebot)

find_package(catkin REQUIRED COMPONENTS message_generation turtlebot3_msgs)

add_message_files(
   FILES
   SensorData.msg
   WheelCommands.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package(CATKIN_DEPENDS message_runtime turtlebot3_msgs)

install(PROGRAMS  scripts/build_firmware
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )

 #build_firmware expects arduino-cli.yaml to be in the same directory so
 # install it to the bin directory
install(FILES scripts/arduino-cli.yaml
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
