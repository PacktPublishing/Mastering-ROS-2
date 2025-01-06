#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  ldlidar"
echo "ldlidar usb connection as /dev/ldlidar , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy ldlidar.rules to  /etc/udev/rules.d/"
sudo cp ./ldlidar.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
echo "finish "
