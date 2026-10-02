FROM ros:jazzy-ros-base-noble

ENV ROS_DISTRO=jazzy
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive

RUN mkdir -p /ws/src

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    libeigen3-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
