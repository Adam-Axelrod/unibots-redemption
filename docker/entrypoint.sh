#!/bin/bash
# Source ROS, and the workspace too if it has been built already.
set -e
source /opt/ros/humble/setup.bash
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi
exec "$@"
