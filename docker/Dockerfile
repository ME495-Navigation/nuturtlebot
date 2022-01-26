# syntax=docker/dockerfile:1.3-labs
FROM ubuntu
RUN apt-get update && apt-get install -y \
  curl \
  parted \
  xz-utils \
  && rm -rf /var/lib/apt-lists/* \
  && mkdir /build

COPY mount_image /build/mount_image

#RUN curl -L "https://cdimage.ubuntu.com/releases/20.04.3/release/ubuntu-20.04.3-preinstalled-server-armhf+raspi.img.xz" | \
#  unxz > /build/raspi.img

# Expand the image
#RUN truncate -s 8GB /build/raspi.img && \
#  parted /build/raspi.img resizepart 2 100% && \
#  mkdir /build/raspi

# Create the loopback device and expand partitions, and mount
#RUN --security=insecure /build/mount_image

