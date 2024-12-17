#!/usr/bin/env bash
set -e

# setup ros environment
source "$ROS_WS/install/setup.bash"

exec "$@"
