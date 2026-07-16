#!/bin/bash
set -e

# Cargar ROS 2 y el workspace
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

# Ejecutar el comando que se le pase al contenedor
exec "$@"