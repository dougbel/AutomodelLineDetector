#!/bin/bash
set -e

# Source ROS and workspace
# TODO it is necesary to run this every single time I enter here
source "/opt/ros/noetic/setup.bash" --
# source /home/docker/catkin_ws/devel/setup.bash

# Execute passed command
exec "$@"
