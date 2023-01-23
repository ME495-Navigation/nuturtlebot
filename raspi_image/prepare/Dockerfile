# Download and prepare a raspberry pi image for mounting
# Mounting a disk image can not be easily done when building
# So first this docker container should be built
FROM ubuntu:jammy

RUN apt-get update && apt-get install -y \
    curl \
    docker.io \
    parted \
    xz-utils

COPY 01-download_raspi_image.sh /usr/bin/
RUN 01-download_raspi_image.sh

COPY 02-expand_raspi_image.sh /usr/bin
RUN 02-expand_raspi_image.sh
RUN mkdir /mnt/raspi

COPY 03-resize_mount.sh /usr/bin
COPY 04-build_base.sh /usr/bin