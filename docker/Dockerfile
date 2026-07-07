FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Update and upgrade system
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
        python3-pip \
        python3-flask \
        python3-opencv \
        libopencv-dev \
        ros-humble-demo-nodes-py \
        ros-humble-tf2 \
        ros-humble-tf2-tools \
        ros-humble-tf2-ros \
        ros-humble-tf2-msgs \
        ros-humble-cv-bridge \
        ros-humble-image-transport \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        v4l-utils \
        usbutils \
        # ros2-testing-apt-source \
        # ros-humble-depthai-ros-v3 \
        && rm -rf /var/lib/apt/lists/*

# Source ROS automatically
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Install standard OpenCV package
RUN pip install --no-cache-dir opencv-python

WORKDIR /root

CMD ["/bin/bash"]

# BUILD CMD
# docker build -t ros2_delta .

# RUN CMD
#docker run -it \
#  --name ros2_delta \
#  --network host \
#  --device=/dev/video0 \
#  --device=/dev/video1 \
#  --device=/dev/ttyUSB0 \
#  --device=/dev/ttyUSB1 \
#  --device=/dev/ttyACM0 \
#  --device=/dev/ttyACM1 \
#  ros2_delta
