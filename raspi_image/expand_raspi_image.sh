#!/bin/sh
# Resize a raspberry pi disk image to expand the system partition

# ----- Environment Variables -----
image=raspi.img   # Name of the raspberry pi image
newsize=8GB       # New size of the image file
# ---------------------

# Increase the size of the whole disk image
truncate -s $newsize $image

# Expand the system partition to take up the new space
parted $image resizepart 2 100% > /dev/null 2>&1
