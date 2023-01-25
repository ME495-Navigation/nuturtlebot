#!/bin/sh
# Create a raspberry pi image suitable for the turtlebot 3

# Get the location of this script
script=$(readlink -f "$0")
script_dir=$(dirname "$script")

echo "Creating the numsr:prep docker image"
#    - This Dockerfile is the build environment for the image
cd $script_dir/prepare
sudo docker build -t numsr:prep .

echo "Starting the prep container in the background"
sudo docker run -t -d \
     --name prep_container \
     -v /dev:/dev \
     -v /var/run/docker.sock:/var/run/docker.sock \
     -v ${script_dir}/raspi/Dockerfile:/mnt/Dockerfile \
     --privileged numsr:prep \
     /bin/bash

echo "Mounting and resizing the raspberry pi image"
sudo docker exec prep_container 03-resize_mount.sh

echo "Creating the base raspi_ubuntu image"
# This image will be available on the host system
# and contains everything the raspberry pi has
# 04-build_base creates a docker container called
# numsr:raspi_ubuntu
sudo docker exec prep_container 04-build_base.sh

echo "Building the numsr:raspi_clean (ubuntu with cruft removed)"
cd ${script_dir}/cleanup
sudo docker build -t numsr:raspi_clean .

echo "Building the numsr:raspi_ros2 (ros2 installation)"
cd ${script_dir}/cleanup
sudo docker build -t numsr:raspi_ros2 .

echo "Building the numsr:turtlebot (turtlebot3 installation)"
cd ${script_dir/turtlebot}
sudo docker build -t numsr:turtlebot .
# Build a bunch of other stuff

# for the final image
# docker export the final image to a tar file
# get the tar file into the prep container
# clear the image in the prep container
