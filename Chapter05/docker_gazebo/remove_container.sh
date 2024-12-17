#!/bin/sh

CONTAINER_NAME="$1" #name of container which you created earlier, by running "create_container.sh" file.

# Running the existing container

docker stop $CONTAINER_NAME
docker rm $CONTAINER_NAME
