#!/bin/bash
# Raspberry Pi only. Bridges the ESP32 onto the ROS graph.
#
# Nothing on the real robot works until this is running: /cmd_vel goes nowhere
# and no /odom_raw appears. Leave it running in its own terminal.
#
# Uses 921600 baud -- that is the micro-ROS link speed. The 115200 in
# config_robot.py is a different protocol on the same cable; the two cannot
# run at once, so stop this before configuring the board.
docker run -it --rm \
  -v /dev:/dev -v /dev/shm:/dev/shm \
  --privileged --net=host \
  --env="ROS_DOMAIN_ID=20" \
  microros/micro-ros-agent:humble \
  serial --dev /dev/ttyUSB0 -b 921600 -v4
