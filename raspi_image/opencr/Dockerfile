# This dockerfile is for building the opencr firmware
FROM ubuntu:jammy

RUN dpkg --add-architecture i386 && \
    apt-get update && \
    apt-get install -y \
    curl \
    libc6:i386 \
    libc++6:i386 \
    git

# Clone (my fork of) the opencr repository
RUN git clone https://github.com/ME495-Navigation/OpenCR -b numsr

# Install the arduino CLI from instructions https://arduino.github.io/arduino-cli/0.29/installation/
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh




# Install the OpenCR board files
COPY arduino-cli.yaml /arduino-cli.yaml

RUN ln -s /OpenCR/arduino/opencr_release /arduino && \
    arduino-cli --config-file /arduino-cli.yaml core install OpenCR:OpenCR -v && \
    arduino-cli --config-file /arduino-cli.yaml core update-index && \
    arduino-cli --config-file /arduino-cli.yaml lib update-index && \
    arduino-cli lib install Dynamixel2Arduino

# Replace the supplied version of OpenCR with my fork of it
RUN version=$(ls --color=never /arduino/packages/OpenCR/hardware/OpenCR/) && \
    echo "Found OpencR Version $version" && \
    rm -rf /arduino/packages/OpenCR/hardware/OpenCR/$version/libraries && \
    ln -s /OpenCR/arduino/opencr_arduino/opencr/libraries arduino/packages/OpenCR/hardware/OpenCR/$version/libraries

# This command creates the firmware in the firmware_stock directory
RUN arduino-cli --config-file /arduino-cli.yaml compile  --build-path firmware_stock \
                --fqbn OpenCR:OpenCR:OpenCR arduino/packages/OpenCR/hardware/OpenCR/1.5.1/libraries/turtlebot3_ros2/examples/turtlebot3_burger/turtlebot3_burger.ino # -v (add for verbose output)

# create Opencr Firmware files
WORKDIR /firmware_stock
RUN /OpenCR/arduino/opencr_develop/opencr_ld_shell/opencr_ld_shell_x86 make turtlebot3_burger.ino.bin burger NU-MSR