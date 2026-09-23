# Raspberry Pi image. Same ROS base as the laptop one, plus the MS200 lidar
# driver.
#
# The driver is vendored as vendor/oradar_ros.tar.xz rather than fetched at
# build time. Yahboom ships it only as a Google Drive download -- their GitHub
# link (YahboomTechnology/MS200Lidar) is a 404 -- and a Drive URL is not a
# reproducible build source. Keeping the tarball here means the image builds
# offline and byte-identically every time.
#
# It is built into its own overlay at /opt/lidar_ws, deliberately outside /ws,
# so the three packages a collaborator reads are still the only things in the
# workspace.
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        ros-humble-tf2-ros \
        xz-utils \
    && rm -rf /var/lib/apt/lists/*

COPY vendor/oradar_ros.tar.xz /tmp/oradar_ros.tar.xz

# The tarball unpacks to oradar_ros/, but the ROS package inside is named
# oradar_lidar. Its shipped package.xml is already the ROS 2 one and its
# CMakeLists already has set(COMPILE_METHOD COLCON), so it builds as-is.
RUN mkdir -p /opt/lidar_ws/src \
    && tar -xJf /tmp/oradar_ros.tar.xz -C /opt/lidar_ws/src \
    && rm /tmp/oradar_ros.tar.xz \
    && . /opt/ros/humble/setup.sh \
    && cd /opt/lidar_ws \
    && colcon build --symlink-install

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo '[ -f /opt/lidar_ws/install/setup.bash ] && source /opt/lidar_ws/install/setup.bash' >> /root/.bashrc && \
    echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
