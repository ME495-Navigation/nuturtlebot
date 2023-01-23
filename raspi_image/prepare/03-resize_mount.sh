#!/bin/sh
# Resize the raspberry pi filesystem and mount it.

loop_dev=$(losetup --show -Pf raspi.img)

#  resize the filesystem so it fills the partition
e2fsck -y -f ${loop_dev}p2
resize2fs ${loop_dev}p2

mount ${loop_dev}p2 /raspi
mount ${loop_dev}p1 /raspi/boot/firmware

