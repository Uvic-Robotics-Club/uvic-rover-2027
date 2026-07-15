#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
if [ -f /rover_ws/install/setup.bash ]; then
  source /rover_ws/install/setup.bash
fi

exec "$@"