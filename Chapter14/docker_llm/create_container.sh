#!/bin/sh


DOCKER_IMAGE="$1"  # Name of the Docker image.
ROS2_WS_NAME="$2"    #ROS2 ws name
CONTAINER_NAME="$3"  # Name of the container you want to create.


HOST_WS_PATH="/home/$USER/$ROS2_WS_NAME/src"  # Path to your workspace on the host.

xhost +local:docker

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
XAUTH_DOCKER=/tmp/.docker.xauth

# Create Xauth if not present
if [ ! -f "$XAUTH" ]; then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f "$XAUTH" nmerge -
    else
        touch "$XAUTH"
    fi
    chmod a+r "$XAUTH"
fi

# Remove existing container
docker rm -f "$CONTAINER_NAME"


# Check for NVIDIA GPU
if nvidia-smi | grep -q NVIDIA; then
    echo "NVIDIA GPU detected, initializing container with GPU support"
    docker run -it --network host \
        --privileged \
        --name "$CONTAINER_NAME" \
        --gpus all \
        --runtime nvidia \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="NVIDIA_DRIVER_CAPABILITIES=all" \
        --volume="/etc/timezone:/etc/timezone:ro" \
        --volume="/etc/localtime:/etc/localtime:ro" \
        --volume="$XSOCK:$XSOCK:rw" \
        --volume="$XAUTH:$XAUTH_DOCKER:rw" \
        --volume="$HOST_WS_PATH:/home/$USER/$ROS2_WS_NAME/src" \
        "$DOCKER_IMAGE" \
        bash
else
    echo "NVIDIA GPU NOT detected, initializing container without GPU support"
    docker run -it --network host \
        --privileged \
        --name "$CONTAINER_NAME" \
        --env="DISPLAY=$DISPLAY" \
        --volume="$XSOCK:$XSOCK:rw" \
        --volume="$XAUTH:$XAUTH_DOCKER:rw" \
        --volume="/dev:/dev" \
        --volume="/etc/timezone:/etc/timezone:ro" \
        --volume="/etc/localtime:/etc/localtime:ro" \
        --volume="$HOST_WS_PATH:/home/$USER/$ROS2_WS_NAME/src" \
        "$DOCKER_IMAGE" \
        bash
fi
