# Nuturtlebot 
Provides low-level access to the Turtlebot3 in ROS. 
Also provides convenient scripts for building the turtlebot3 OpenCR firmware (and modified versions thereof) from the command line.

1. Access to raw sensor data (e.g., encoder ticks)
2. Access to raw motor commands (e.g., directly control wheel velocities).

The purpose is to enable learning low-level robot control in ROS without needing to do any embedded programming
(even though, outside an educational context, it makes sense to perform the conversion between say `cmd_vel` messages and
wheel velocities directly on the OpenCR embedded microcontroller, as is the default for ROBOTIS).

Must be used in conjunction with the raw OpenCR firmware, which can be built using scripts in this repository.

# Building the Firmware
These are instructions for how to build the firmware.  
*If your turtlebot3 is already setup, you need not do run these commands*

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
