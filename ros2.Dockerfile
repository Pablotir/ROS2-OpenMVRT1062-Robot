# ROS 2 Iron for Ubuntu 22.04 (JetPack 6+)
ARG ROS_DISTRO=iron
FROM ros:${ROS_DISTRO}-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# System dependencies including ros-iron-vision-opencv
RUN apt-get update && apt-get install -y \
    python3-pip python3-opencv ffmpeg libsm6 libxext6 \
    libatlas-base-dev libopenblas-dev libhdf5-dev \
    build-essential cmake git curl nano tmux \
    libusb-1.0-0-dev libjpeg-dev \
    ros-iron-vision-opencv \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Python libraries with correct versions
RUN python3 -m pip install --no-cache-dir \
    "opencv-python-headless" \
    "numpy<2,>=1.26" \
    pyserial pillow apriltag requests

# Workspace setup
ENV ROS_WS=/root/ros2_ws
WORKDIR $ROS_WS
RUN mkdir -p src

# Copy workspace and entrypoint
COPY ./src ./src
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Use bash login shell to allow sourcing
SHELL ["/bin/bash", "-lc"]

# Add ROS2 source to bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]

