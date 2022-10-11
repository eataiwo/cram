#!/bin/bash

# exit immediately if a command exits with a non-zero status (see `$ help set`)
set -e

ECHO_PREFIX="[docker-entrypoint.sh]"

# ROS installation
ROS=/opt/ros/noetic/setup.bash
source "$ROS"
echo "$ECHO_PREFIX" "sourced ROS installation:" "$ROS"

# workspace holding custom ROS packages
workspace=/catkin_ws
source "$workspace"/devel/setup.bash
echo "$ECHO_PREFIX" "sourced workspace:" "$workspace"

echo "$ECHO_PREFIX" "execute" "$@"
exec "$@"