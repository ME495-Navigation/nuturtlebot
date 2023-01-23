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
     --privileged numsr:prep \
     /bin/bash

echo "Mounting and resizing the raspberry pi image"
sudo docker exec prep_container 03-resize_mount.sh 
