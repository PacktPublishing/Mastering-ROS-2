#!/usr/bin/env bash

echo "Stopping running containers"
docker stop $(docker ps -q)

echo "Deleting containers"
docker container prune

echo "Deleting all images"
docker system prune -a