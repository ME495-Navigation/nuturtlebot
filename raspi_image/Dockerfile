# Creates a raspberry pi image suitable for use with a turtlebot
FROM ubuntu:jammy



RUN apt-get update -yq && \
    apt-get install -yq \
    bash \
    curl \
    fdisk \
    p7zip-full \
    xz-utils
RUN mkdir raspi && \
    curl -L https://cdimage.ubuntu.com/releases/22.04.1/release/ubuntu-22.04.1-preinstalled-server-arm64+raspi.img.xz > /raspi/raspi.img.xz

# Extract the disk images
# Get the boot partition and write it to a file. Assumes a 512 byte sector size
# Th sfdisk command is printing the partition table, getting the end size in sectors of the first partition
# dd is then extracting the appropriate part of the file
RUN cd /raspi && \
    unxz raspi.img.xz && \
    dd if=raspi.img of=boot.img bs=512 \
       skip=$(sfdisk -l raspi.img | tail -n 2 | awk '{ print $3; }' | head -n 1) \
       count=$(sfdisk -l raspi.img | tail -n 2 | awk '{ print $4; }' | head -n 1) && \
    dd if=raspi.img of=part.img bs=512 skip=$(sfdisk -l raspi.img | tail -n 2 | awk '{ print $4+1; }' | head -n 1)

# 