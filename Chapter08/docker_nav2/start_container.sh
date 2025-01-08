#!/bin/sh

CONTAINER_NAME="$1" #name of container which you created earlier, by running "create_container.sh" file.

# Enable access control for X server to avoid GUI issues
xhost +local:docker

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
XAUTH_DOCKER=/tmp/.docker.xauth

# Create Xauth if not present
if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Running the existing container
docker start $CONTAINER_NAME
docker exec -it $CONTAINER_NAME bash
