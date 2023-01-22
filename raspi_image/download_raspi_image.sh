#!/bin/sh
# Download the raspberry pi image and unzip it

curl -L https://cdimage.ubuntu.com/releases/22.04.1/release/ubuntu-22.04.1-preinstalled-server-arm64+raspi.img.xz > raspi.img.xz
unxz raspi.img.xz
