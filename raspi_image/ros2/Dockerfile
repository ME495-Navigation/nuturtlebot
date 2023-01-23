from numsr:raspi_clean

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg]" \
         "http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list  && \
         apt-get update && \
         apt-get install -y \
                 build-essential \
                 debhelper \
                 dpkg-dev \
                 ffmpeg \
                 gdb \
                 git \
                 network-manager \
                 libarmadillo-dev \
                 libeigen3-dev \
                 libopencv-dev \
                 python3-bloom \
                 python3-numpy \
                 python3-scipy \
                 python3-sympy \
                 python3-serial \
                 python3-rosdep \
                 python3-vcstool \
                 ros-humble-ros-base \
                 ros-dev-tools \
                 ssh \
                 v4l-utils
RUN rosdep init
RUN systemctl enable ssh