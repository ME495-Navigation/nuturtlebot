#!/bin/sh
# The raspberry pi disk is divided as follows:
# All units are in sectors
# There are 512 bytes per sector

# The path to the image file
image=raspi.img

# The size of each sector
sector_size=512

# Starting sector of the boot partition
boot_start=$(sfdisk -l -o Start $image | tail -n 2 | awk '{ print $1 }' | head -n 1)

# The partition table is in sector 0 - (boot_start-1), and therefore it is boot_start bytes long
# (since indexing starts from zero)
part_count=$boot_start

# The number of sectors in the boot partition
boot_count=$(sfdisk -l -o Sectors $image | tail -n 2 | awk '{ print $1 }' | head -n 1)

echo $boot_start
echo $boot_count
# The start of the system partition
sys_start=$(sfdisk -l -o Start $image | tail -n 2 | awk '{ print $1 }' | tail -n 1)

# The number of sectors of the system partition
sys_count=$(sfdisk -l -o Sectors $image | tail -n 2 | awk '{ print $1 }' | tail -n 1)

# Extract the partition table
dd if=raspi.img of=mbr.img bs=$sector_size \
   count=$part_count

# Extract the boot partition.
dd if=raspi.img of=boot.img bs=$sector_size \
       skip=$boot_start \
       count=$boot_count

# Extract the system image
dd if=raspi.img of=sys.img bs=512 \
   skip=$sys_start \
   count=$sys_count
