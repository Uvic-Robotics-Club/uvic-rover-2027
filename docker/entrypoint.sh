#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
if [ -f /rover_ws/install/setup.bash ]; then
  source /rover_ws/install/setup.bash
fi

# Install any missing ROS dependencies declared in package.xml files
if [ -d /rover_ws/src ]; then
  sudo apt-get update
  rosdep update && rosdep install --from-paths /rover_ws/src --ignore-src -r -y
fi

exec "$@"