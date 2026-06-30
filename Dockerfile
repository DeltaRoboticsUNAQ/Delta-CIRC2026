FROM ros:ros-humble-base

ENV DEBIAN_FRONTEND=noninteractive

# Update and upgrade system
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
        python3-pip \
        python3-flask \
        ros-humble-demo-nodes-py \
        ros-humble-rviz2 \
        ros-humble-tf2 \
        ros-humble-tf2-tools \
        ros-humble-tf2-ros \
        ros-humble-tf2-msgs \
        && rm -rf /var/lib/apt/lists/*

# Source ROS automatically
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /root

CMD ["/bin/bash"]