# Nuturtlebot 
Provides low-level access to the Turtlebot3 in ROS. 
Also provides convenient scripts for building the turtlebot3 OpenCR firmware (and modified versions thereof) from the command line.

1. Access to raw sensor data (e.g., encoder ticks)
2. Access to raw motor commands (e.g., directly control wheel velocities).

The purpose is to enable learning low-level robot control in ROS without needing to do any embedded programming
(even though, outside an educational context, it makes sense to perform the conversion between say `cmd_vel` messages and
wheel velocities directly on the OpenCR embedded microcontroller, as is the default for ROBOTIS).

Must be used in conjunction with the raw OpenCR firmware, which can be built using scripts in this repository.

# Usage
1. There are two types of users: Those who need ROS access to raw turtlebot signals, and those building custom turtlebot3 images
2. If you need access to raw turtlebot signals, and your turtlebot already has the appropriate firmware installed, then this package
   just provides some message types, and you only need to have it on your ROS path (either by cloning it into your workspace or installing it).
3. Read on if you'd like to build the firmware or the custom Northwestern turtlebot image (NU/MSR students need not do these steps!)

# Building the Nuturtlebot Image
These instructions explain how to build a  custom SDCard  image for the turtlebot3.
You need to be root to do this. It is a TODO to lower the permission requirements
**If your turtlebot3 is already set up, you need not run these commands**
0. Install the prequisites: =apt install qemu-user-static proot binfmt-support xz-utils=
1. Run [scripts/create_image](scripts/create_image) to create the basic image.
   - Can be run as `rosrun nuturtlebot create_image` and will create a file called `build/turtlebot_image/turtlebot.img` that can be written to the SD card.
   - This script automatically builds both versions of the opencr firmware
2. Run [scripts/write_image](scripts/write_image) to write the image to the sd_card.
   - This script allows each image written to be customized for a specific robot
   - Should be invoked as `rosrun nuturtlebot write_image <image> <sdcard> <name> <pubkey> <nmcon> [robotdns]`
     - `<image>`: Path to the image file (e.g., `build/turtlebot_image/turtle.img`)
     - `<sdcard>`: Path to the sdcard device (e.g. `/dev/mmcblk0`)
     - `<name>`: The hostname for the turtlebot. Each turtlebot on your network should have a unique hostname
     - `<pubkey>`: An ssh public key that will let the people with the corresponding private key to logon as the `msr` user
     - `<nmcon>`: A network-manager connection file, corresponding to the network the turtlebot should connect to.
       - Can specify the full path or will search for a file called `/etc/NetworkManager/system-connections/<nmcon>.nmconnection`
       - If you are connected to the same wifi network you want to use, <nmcon> can usually be the SSID of that network
     - `<robotdns>` (optional): The [robotdns](https://github.com/m-elwin/robotdns) server location, if being used.
       - robotdns is a dynamic DNS server that allows all robots to communicate via hostnames even when we do not control the LAN settings (e.g., are on a corporate network).
       - For this option to work, the `<nmcon>` must be a network profile created by robotdns and be named `network.robot.nmconnection`.
         
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
