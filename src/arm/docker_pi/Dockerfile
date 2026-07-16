FROM ros:jazzy-ros-base
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip python3-serial libhidapi-hidraw0 libhidapi-libusb0 \
    ros-jazzy-sensor-msgs ros-jazzy-std-msgs ros-jazzy-control-msgs ros-jazzy-trajectory-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install hid --break-system-packages

WORKDIR /ros2_ws
# Copia el código real desde la carpeta anterior sin duplicarlo
COPY . /ros2_ws/src

# Construye el workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Copia el script de inicio desde tu nueva subcarpeta
COPY docker_pi/entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]