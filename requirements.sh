#!/bin/bash

echo "Updating system and installing ROS dependencies..."
sudo apt-get update
sudo apt-get install -y \
    ros-noetic-depth-image-proc \
    ros-noetic-nodelet \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-octomap-server \
    
echo "Installing Python dependencies..."
pip3 install numpy opencv-python

echo "All dependencies have been installed!"
