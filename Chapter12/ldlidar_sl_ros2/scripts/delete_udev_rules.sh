#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to  ldlidar"
echo "sudo rm   /etc/udev/rules.d/ldlidar.rules"
sudo rm   /etc/udev/rules.d/ldlidar.rules
echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
echo "finish  delete"
