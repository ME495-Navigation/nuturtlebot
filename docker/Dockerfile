FROM scratch
COPY raspi_root /

COPY sources.list /etc/apt/sources.list
COPY qemu-arm-static /usr/bin/
COPY dependencies.txt /dependencies.txt


RUN dpkg --add-architecture amd64 && apt-get update

# Remove packages that aren't useful for cross-compiling
# And that can cause problems since there is no kernel
RUN apt-get purge -yq \
    initramfs-tools \
    linux-image* \
    linux-modules* \
    man-db \
    u-boot-rpi && \
    apt-get -yq autoremove 


# smoothly installing amd64 packages
RUN apt-get install --allow-remove-essential -yq \
  coreutils:amd64 \
  bash:amd64 \
  dash:amd64 \
  gawk:amd64 \
  grep:amd64 \
  sed:amd64 \
  findutils:amd64 \
  make:amd64

# Install cmake, but I need to patch a dependency on cmake-data, since it tries to dl a non-existant amd:64 version
RUN apt-get download cmake:amd64 &&\
   dpkg-deb -R cmake*.deb cmake && \
   sed -i s/cmake-data/cmake-data:armhf/ cmake/DEBIAN/control &&\
   dpkg-deb -b cmake cmake.deb && \
   apt-get install -yq ./cmake.deb && \
   rm -rf cmake cmake*.deb

# Manually download and install all the x86 packages we need while ignoring dependencies.
# The binaries will overwrite the arm binaries but the
# amd64 and armhf libraries will remain installed side by side
# Ideally, all uses of the docker container after this point
# including during the build process, will not require qemu-arm-static

# A list of all recursive dependencies generated running the command on a working Ubuntu system
# apt-rdepends \
#              gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf\
#              python3 lsb-release perm passwd | grep -v "^ "\
#              | grep -v "^awk$" | grep -v "debconf-2.0" > dependencies.txt
RUN apt-get install -yq --allow-remove-essential apt:amd64 
RUN apt-get download $(xargs -a dependencies.txt)
RUN dpkg -i --force-all *.deb && rm *.deb

# We now have most of our build tools running native
# Bring in the cmake toolchain file
COPY raspi-toolchain.cmake /toolchain.cmake
RUN mkdir /ros_ws
VOLUME /ros_ws

COPY catkin_arm /usr/bin/catkin_arm
COPY catkin_arm_isolated /usr/bin/catkin_arm_isolated
COPY arm /arm
RUN chmod 666 /etc/group /etc/passwd
WORKDIR /ros_ws
# replace the container with the container id
# which the hostname is set to
CMD ["cat", "/arm"]