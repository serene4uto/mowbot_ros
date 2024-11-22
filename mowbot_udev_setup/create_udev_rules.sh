#!/bin/bash

echo "remap the device serial ports of mowbot"
echo "start copy mowbot_udev.rules to /etc/udev/rules.d/"
sudo cp mowbot_udev.rules  /etc/udev/rules.d
echo -e "\nRestarting udev\n"
sudo service udev reload
sudo service udev restart
sudo udevadm control --reload && sudo udevadm trigger
echo "finish"
