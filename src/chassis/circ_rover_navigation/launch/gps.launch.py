#!/usr/bin/env python3
"""
gps.launch.py  ──  circ_rover_navigation/launch/

Stack de localización GLOBAL (anclaje GPS). Levanta:

  1. nmea_serial_driver  -> lee el GPS, publica /fix
  2. navsat_transform    -> /fix -> /odometry/gps (posición en frame map)
  3. ekf_global          -> fusiona ruedas+IMU+GPS, publica TF map -> odom
  4. static tf base_link -> gps  (posición de la antena en el chasis)

Corre ESTE launch JUNTO con localization.launch.py (que da el ekf_local).
La IMU (/imu/data_raw) debe estar publicando.

Uso:
  ros2 launch circ_rover_navigation gps.launch.py
  ros2 launch circ_rover_navigation gps.launch.py gps_port:=/dev/ttyACM0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('circ_rover_navigation')
    ekf_yaml    = os.path.join(nav_share, 'config', 'ekf_global.yaml')
    navsat_yaml = os.path.join(nav_share, 'config', 'navsat_transform.yaml')

    gps_port = LaunchConfiguration('gps_port')
    declare_port = DeclareLaunchArgument('gps_port', default_value='/dev/ttyACM0')

    # ── 1: driver del GPS (publica /fix) ──
    gps_driver = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='nmea_driver',
        parameters=[{'port': gps_port, 'baud': 9600, 'frame_id': 'gps'}],
        output='screen',
    )

    # ── 2: navsat_transform (GPS -> frame map) ──
    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        parameters=[navsat_yaml],
        remappings=[
            ('imu', '/imu/data_raw'),               # entrada: IMU
            ('gps/fix', '/fix'),                    # entrada: GPS
            ('odometry/filtered', '/odometry/global'),  # entrada: pose del ekf_global
            # salidas (nombres por defecto): /odometry/gps  y  /gps/filtered
        ],
        output='screen',
    )

    # ── 3: EKF global (publica /odometry/global y el TF map->odom) ──
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        parameters=[ekf_yaml],
        remappings=[('odometry/filtered', '/odometry/global')],
        output='screen',
    )

    # ── 4: posición de la antena GPS en el chasis (ajusta x,y,z reales) ──
    gps_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_gps',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps'],
        output='screen',
    )

    return LaunchDescription([
        declare_port,
        gps_driver,
        navsat,
        ekf_global,
        gps_tf,
    ])