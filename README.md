# Nuturtlebot
## Overview
Enable learning low-level robot control in ROS without needing to do any embedded programming
(even though, outside an educational context, it makes sense to perform the conversion between say `cmd_vel` messages and
wheel velocities directly on the OpenCR embedded microcontroller, as is the default for ROBOTIS).

## Contents
1. Scripts for building a raspberrypi 3 Ubuntu image for use with the turtlebot
   - Contains all ROS packages for running the turtlebot3
   - Ability to switch between original OpenCR firmware and custom raw firmware
   - Custom Raw firmware enables ROS access to raw sensor data (e.g., encoder ticks) and direct motor control of wheel velocities
2. Scripts for building turtlebot3 OpenCR firmware (and modified versions) from the command line
   - Uses the Arduino CLI interface to build OpenCR firmware, from a custom fork of that firmware
3. Docker container for cross-compiling programs that run on the Raspberry PI

# Setup Cross-Compiling
1. Install docker and pre-requisites: `sudo apt install docker.io`
2. In the base of the ROS workspace run:
   `docker run reem17/rosberrypi:noetic > arm && chmod 755 arm`
3. To build with `catkin_make` for arm use `./arm catkin_arm`
3. To build with `catkin_make_isolated` for arm use `./arm catkin_arm_isolated`

# Building the Nuturtlebot Image
These instructions explain how to build a  custom SDCard  image for the turtlebot3.
You need to be root to do this. It is a TODO to lower the permission requirements
**If your turtlebot3 is already set up, you need not run these commands**

1. Install the prequisites: `apt install qemu-user-static binfmt-support xz-utils`
1. Run [scripts/create_image](scripts/create_image) to create the basic image.
   - Can be run as `rosrun nuturtlebot create_image` and will create a file (relative to the current directory) 
     called `build/turtlebot_image/turtlebot.img` that can be written to the SD card.
   - This script automatically builds both versions of the OpenCr firmware
3. After writing the image to the SDcard, the raspberry pi should boot and connect to the network.  ssh into the raspberry pi, then
   run `opencr_update stock` to install the stock firmware or `opencr_update raw` to install the low-level firmware

2. If your turtlebot3 has not yet been setup, run [scripts/write_image](scripts/write_image) to write the image to the sd_card
   - This script allows each image written to be customized for a specific robot
   - Should be invoked as `rosrun nuturtlebot write_image <sdcard> <name> <pubkey> <nmcon> [robotdns]`
     - `<sdcard>`: Path to the sdcard device (e.g. `/dev/mmcblk0`)
     - `<name>`: The hostname for the turtlebot. Each turtlebot on your network should have a unique hostname
     - `<pubkey>`: An ssh public key that will let the people with the corresponding private key to logon as the `msr` user
     - `<nmcon>`: A network-manager connection file, corresponding to the network the turtlebot should connect to.
       - Can specify the full path or will search for a file called `/etc/NetworkManager/system-connections/<nmcon>.nmconnection`
       - If you are connected to the same wifi network you want to use, <nmcon> can usually be the SSID of that network. The profile
         will be written to the SDCard in a way that will work with the target hardware.
       - Computer-specific parts of the file will stripped from the connection profile prior to installation on the turtlebot
     - `<robotdns>` (optional): The [robotdns](https://github.com/m-elwin/robotdns) server location, if being used.
       - robotdns is a dynamic DNS server enabling robots to communicate via hostnames even when we do not control the LAN settings (e.g., are on a corporate network).
       - For this option to work, the `<nmcon>` must be a network profile created by the robotdns setup and be named `network.robot.nmconnection`.
3. Log into the turtlebot and install the firmware with either `opencr_update stock` or `opencr_update raw` (see below).       
# Building the Firmware
These are instructions for how to build the firmware, which is automatically done when building the SDCard image, but can be done separately.  
**If your turtlebot3 is already set up, you need not run these commands**

1. Clone the repository into a workspace
2. Install the dependencies: `rosdep --install --from-paths src --ignore-src -r -y`
3. Build the workspace (`catkin_make` or `catkin build`, `catkin_make_isolated` is not supported currently)
4. `rosrun nuturtlebot build_firmware`
   - This command will download arduino-cli tools and build both the stock firmware and the raw firmware
   - The stock firmware is stored in `build/opencr/burger.opencr` (Put it on the turtlebot3 at `/usr/share/opencr`)
   - The raw firmware is stored in `build/opencr/raw-burger.opencr`(Put it on the turtlebot3 at `/usr/share/opencr`
   - The firmware loader (to be run on the turtlebot) is in `build/opencr/opencr_ld_shell` (Put it on the turtlebot3 at `/usr/bin`).
   - The firmware update wrapping script is in `build/opencr/opencr_update` (Put in on the turtlebot3 at `/usr/bin`)
   - Use `opencr_update stock` on the turtlebot3 to install stock firmware and `opencr_update raw` to install the raw firmware

# Building The Docker Cross-compile image
- The Docker Cross-compile image is a docker image that is based off of the raspberry pi image.
- The libraries on the docker cross-compile image are therefore the same as those on the raspberry pi
- However, the dockerfile replaces several armhf binaries with amd64 binaries and installs a cross compiler
- First, build the turtlebot image
- Then, run `./scripts/docker_build`
