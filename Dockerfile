# Use the official ROS 2 Humble desktop image from OSRF (includes RViz).
FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-opencv \
    python3-numpy \
    python3-yaml \
    ros-humble-cv-bridge \
    ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws
COPY . /ros_ws/src/self_localization

RUN source /opt/ros/humble/setup.bash \
    && colcon build --packages-select self_localization

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /ros_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
