#!/bin/bash
# Build the firmware in Docker. Works on any laptop; needs no ESP-IDF install.
# micro-ROS is cloned at a pinned commit rather than copied into the repo.
set -euo pipefail
cd "$(dirname "$0")"
UROS_COMMIT=b5eee69

if [ ! -d components/micro_ros_espidf_component ]; then
    git clone -q -b humble https://github.com/micro-ROS/micro_ros_espidf_component.git \
        components/micro_ros_espidf_component
fi
git -C components/micro_ros_espidf_component checkout -q "$UROS_COMMIT"

docker build -q -t unibots-firmware-builder . > /dev/null
docker run --rm -v "$PWD:/project" -w /project unibots-firmware-builder \
    bash -c "source /opt/esp/idf/export.sh > /dev/null && git config --global --add safe.directory '*' && idf.py build"
