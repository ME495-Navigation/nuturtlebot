from numsr:raspi_ros2

# Setup turtlebot 3
# Follow advanced instructions from
# https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup

RUN systemctl mask systemd-networkd-wait-online.service
RUN systemctl mask sleep.target suspend.target hibernate.target hybrid sleep.target

RUN apt-get update && \
    apt-get install -y \
    ros-humble-hls-lfcd-lds-driver \
    ros-humble-turtlebot3-msgs \
    ros-humble-dynamixel-sdk \
    ros-humble-turtlebot3

RUN . /opt/ros/humble/setup.sh && cp $(ros2 pkg prefix turtlebot3_bringup)/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules /etc/udev/rules.d/