#!/bin/sh
# Build a docker container based off the raspi image
# Because this uses the docker daemon of the host, the host will be able to access this image
cd /mnt
docker build -t numsr:raspi_ubuntu .
