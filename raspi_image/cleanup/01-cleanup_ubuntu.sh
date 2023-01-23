#!/bin/sh

#Remove some annoying ubuntu programs, including snaps
apt-get purge -y unattended-upgrades snapd update-notifier-common lvm2 sosreport cloud-init netplan.io

# Remove snaps
rm -rf /snap /var/snap /var/lib/snapd
cd /etc/systemd/system
ls *snap* | xargs systemctl disable
rm *snap*
cd multi-user.target.wants
rm *snap*
rm *lxd*

# disable motd
systemctl disable motd-news.timer  apt-daily-upgrade.timer apt-daily.timer
rm -rf etc/update-motd.d/*

# make sure this is not accidentally deleted
apt-mark manual squashfs-tools \
         dmeventd \
         libdevmapper-event1.02.1 \
         python3-debian \
         python3-debconf \
         python3-ptyprocess \
         python3-pexpect

apt-get autoremove -y
apt-get upgrade -y

rm -rf /etc/cloud/cloud.cfg.d

apt-get clean
rm -rf /var/lib/apt/lists/*
