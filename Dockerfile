FROM moveit/moveit2:jazzy-release

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y --no-install-recommends \
        rapidjson-dev \
        ros-jazzy-ros2-control* \
        ros-jazzy-serial-driver \
        ros-jazzy-asio-cmake-module \
        ros-jazzy-librealsense2 \
        ros-jazzy-tf-transformations \
        python3-venv \
        htop x11-apps \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /opt/ros/jazzy/env
RUN /opt/ros/jazzy/env/bin/pip install -U opencv-python opencv-contrib-python scipy pyyaml transforms3d numpy==1.*
WORKDIR /workspace
