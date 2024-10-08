#!/bin/bash -eu

# The BSD License
# Copyright (c) 2024 RUNTIME Robotics

#set -x

echo "################################################################"
echo ""
echo ">>> {Uninstalling ROS Jazzy Installation from your computer}"
echo ""
echo ">>> {It will take around few minutes to complete}"
echo ""
sudo apt purge -y ros-jazzy-*
echo ""
echo "#################################################################"
echo ""
echo ">>> {Auto removing dependent packages}"
sudo apt -y autoremove
echo ""
echo ">>> {Done: ROS Jazzy Uninstall}"
echo "#################################################################"

