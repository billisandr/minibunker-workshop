#!/usr/bin/env bash
# Container entrypoint: source ROS + the built workspace, then exec the command
# (defaults to the sim launch via the Dockerfile CMD).
set -e
source /opt/ros/noetic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"
exec "$@"
