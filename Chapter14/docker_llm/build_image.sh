#!/bin/bash
# This script is to create a docker image from the Dockerfile.

# Set the image name as a variable here
IMAGE_NAME="$1"  #Setting the name of the image as argument
ROS_WS="$2"  #Setting name of the host PC ROS workspace folder
USER_NAME="$3"  #Assigning the host user name to the docker  


# Build the Docker image with the specified image name
docker build -f Dockerfile.master_ros2 -t "$IMAGE_NAME" . --build-arg docker_user_name="$USER_NAME" --build-arg ros_ws="$ROS_WS"

