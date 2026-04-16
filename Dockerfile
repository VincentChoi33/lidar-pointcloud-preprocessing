FROM ros:noetic-ros-core

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3 \
    ros-noetic-catkin \
    python3-catkin-tools \
    ros-noetic-nodelet \
    ros-noetic-pluginlib \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-roslaunch \
    libpcl-dev libeigen3-dev \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
RUN mkdir -p /ws/src

CMD ["bash"]
