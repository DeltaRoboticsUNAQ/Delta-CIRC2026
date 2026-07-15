#!/usr/bin/env python3
"""
bringup.launch.py  ──  circ_rover_navigation/launch/

Launch MAESTRO: levanta todo el rover de un solo comando.

Interruptores:
  use_sim:=true   -> motores simulados (fake_motors)
  use_sim:=false  -> bridge real al STM32 de encoders
  gps:=true       -> añade el stack global (GPS + navsat + ekf_global)
  gps:=false      -> solo localización local

Puertos (con udev quedan fijos: /dev/rover_enc, /dev/rover_imu, /dev/rover_gps):
  enc_port, imu_port, gps_port

Ejemplo hardware completo:
  ros2 launch circ_rover_navigation bringup.launch.py use_sim:=false gps:=true \\
       enc_port:=/dev/ttyACM0 imu_port:=/dev/ttyACM2 gps_port:=/dev/ttyACM1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('circ_rover_navigation')
    cfg = lambda name: os.path.join(nav_share, 'config', name)

    use_sim  = LaunchConfiguration('use_sim')
    gps      = LaunchConfiguration('gps')
    enc_port = LaunchConfiguration('enc_port')
    imu_port = LaunchConfiguration('imu_port')
    gps_port = LaunchConfiguration('gps_port')

    args = [
        DeclareLaunchArgument('use_sim',  default_value='true'),
        DeclareLaunchArgument('gps',      default_value='true'),
        DeclareLaunchArgument('enc_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyACM2'),
        DeclareLaunchArgument('gps_port', default_value='/dev/ttyACM1'),
    ]

    # ───────────────── CAPA HARDWARE / SENSORES ─────────────────
    imu_bridge = Node(
        package='circ_rover_hardware', executable='stm32_imu_bridge',
        name='imu_bridge', output='screen',
        parameters=[{'port': imu_port, 'baud': 460800}],   # <-- AHORA SÍ recibe el puerto
    )
    imu_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_to_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
    )

    # ───────────────── MOTORES: simulado Ó real ─────────────────
    fake_motors = Node(
        package='circ_rover_hardware', executable='fake_motors',
        name='fake_motors', output='screen',
        condition=IfCondition(use_sim),
    )
    wheel_bridge = Node(
        package='circ_rover_hardware', executable='wheel_bridge',
        name='wheel_bridge', output='screen',
        parameters=[{'port': enc_port, 'baud': 115200}],
        condition=UnlessCondition(use_sim),
    )

    # ───────────────── CINEMÁTICA + EKF LOCAL ─────────────────
    base = Node(
        package='circ_rover_navigation', executable='skid_steer_base',
        name='skid_steer_base', output='screen',
        parameters=[cfg('rover_base.yaml')],
    )
    ekf_local = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_local', output='screen',
        parameters=[cfg('ekf_local.yaml')],
    )

    # ───────────────── STACK GLOBAL GPS (condicional) ─────────────────
    gps_driver = Node(
        package='nmea_navsat_driver', executable='nmea_serial_driver',
        name='nmea_driver', output='screen',
        parameters=[{'port': gps_port, 'baud': 9600, 'frame_id': 'gps'}],
        condition=IfCondition(gps),
    )
    navsat = Node(
        package='robot_localization', executable='navsat_transform_node',
        name='navsat_transform', output='screen',
        parameters=[cfg('navsat_transform.yaml')],
        remappings=[
            ('imu', '/imu/data_raw'),
            ('gps/fix', '/fix'),
            ('odometry/filtered', '/odometry/global'),
        ],
        condition=IfCondition(gps),
    )
    ekf_global = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_global', output='screen',
        parameters=[cfg('ekf_global.yaml')],
        remappings=[('odometry/filtered', '/odometry/global')],
        condition=IfCondition(gps),
    )
    gps_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_to_gps',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps'],
        condition=IfCondition(gps),
    )

    return LaunchDescription(args + [
        imu_bridge, imu_tf,
        fake_motors, wheel_bridge,
        base, ekf_local,
        gps_driver, navsat, ekf_global, gps_tf,
    ])