#!/bin/bash
# Source ROS, the lidar driver overlay (robot image only), and the workspace
# if it has been built already.
set -e
source /opt/ros/humble/setup.bash
if [ -f /opt/lidar_ws/install/setup.bash ]; then
    source /opt/lidar_ws/install/setup.bash
fi
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi
exec "$@"
