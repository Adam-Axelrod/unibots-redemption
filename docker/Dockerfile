# ROS 2 Humble, headless. No GUI packages: the visualiser is a web page, so we
# never need X11 -- which is what makes this work the same on Mac, Windows and
# Linux. Official images are built for both amd64 and arm64, so Apple Silicon
# runs this natively rather than under emulation.
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

# Every shell in this container gets ROS already sourced -- including the ones
# opened with `docker compose exec`, which skip the entrypoint.
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
