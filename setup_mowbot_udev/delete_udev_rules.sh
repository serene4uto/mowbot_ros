#!/bin/bash

echo "delete remap the device serial ports of mowbot"
sudo rm   /etc/udev/rules.d/mowbot_udev.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"
