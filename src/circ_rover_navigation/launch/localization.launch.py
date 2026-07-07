#!/usr/bin/env python3
"""
localization.launch.py  ──  circ_rover_navigation/launch/

Bringup de localización local. Levanta de un solo comando:

  1. wheel_bridge              -> habla con el STM32 que lee AMBOS encoders
  2. skid_steer_base           -> cmd_vel <-> ruedas, encoders -> /wheel/odometry
  3. ekf_local                 -> fusiona /wheel/odometry + /imu/data_raw
                                  y publica el TF  odom -> base_link

Falta corriendo aparte (o lo añades abajo): el bridge de IMU, que publica
/imu/data_raw. Sin la IMU, el EKF no tiene rumbo.

Uso:
  ros2 launch circ_rover_navigation localization.launch.py
  # o sobreescribiendo el puerto del STM32 de encoders:
  ros2 launch circ_rover_navigation localization.launch.py enc_port:=/dev/ttyACM2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('circ_rover_navigation')
    ekf_yaml  = os.path.join(nav_share, 'config', 'ekf_local.yaml')
    base_yaml = os.path.join(nav_share, 'config', 'rover_base.yaml')

    # ── Puerto del STM32 de encoders (overrideable desde la línea de comandos) ──
    enc_port = LaunchConfiguration('enc_port')
    declare_port = DeclareLaunchArgument('enc_port', default_value='/dev/ttyACM1')

    # ── 1: bridge serie único (lee ambos encoders, manda ambos setpoints) ──
    bridge = Node(
        package='circ_rover_hardware',
        executable='wheel_bridge',          # <- nombre del entry_point en setup.py
        name='wheel_bridge',
        parameters=[{'port': enc_port, 'baud': 115200}],
        output='screen',
    )

    # ── 3: cerebro cinemático (carga rover_base.yaml) ──
    base = Node(
        package='circ_rover_navigation',
        executable='skid_steer_base',
        name='skid_steer_base',
        parameters=[base_yaml],
        output='screen',
    )

    # ── 4: EKF local (carga ekf_local.yaml; el nombre debe ser 'ekf_local') ──
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        parameters=[ekf_yaml],
        output='screen',
    )

    # ── (Opcional) bridge de IMU: descoméntalo y ajusta el executable real ──
    # imu = Node(
    #     package='circ_rover_hardware',
    #     executable='stm32_imu_bridge',
    #     name='imu_bridge',
    #     parameters=[{'port': '/dev/ttyACM0', 'baud': 460800}],
    #     output='screen',
    # )

    return LaunchDescription([
        declare_port,
        bridge,
        base,
        ekf,
        # imu,
    ])