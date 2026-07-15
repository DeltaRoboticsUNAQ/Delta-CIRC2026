#!/usr/bin/env python3
"""
localization.launch.py  ──  circ_rover_navigation/launch/

Bringup de localizacion local. Levanta de un solo comando:

  1. rover_base_node   -> cmd_vel <-> ruedas (cinematica skid-steer),
                          lee los encoders del UNICO STM32 y publica
                          /wheel/odometry.  (Fusiona los antiguos
                          wheel_bridge + skid_steer_base.)
  2. ekf_local         -> fusiona /wheel/odometry (+ /imu/data_raw si hay IMU)
                          y publica el TF  odom -> base_link.

NOTA IMU: sin la IMU, ekf_local corre igual pero SOLO con las ruedas: la
posicion de avance (vx) es valida, pero el rumbo (yaw) no se actualiza al
girar, porque venia del giroscopio. No crashea, solo va degradado.

Este launch REEMPLAZA correr rover_base_node por separado. No lances ambos:
pelearian por el puerto serie.

Uso:
  # con el symlink creado (recomendado): sudo ln -sf /dev/ttyACM0 /dev/rover_enc
  ros2 launch circ_rover_navigation localization.launch.py

  # o apuntando directo al puerto de hoy en WSL2, sin symlink:
  ros2 launch circ_rover_navigation localization.launch.py enc_port:=/dev/ttyACM0
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

    # ── Puerto del STM32 de encoders (overrideable desde la linea de comandos) ──
    # Default al symlink de produccion; en WSL2 crea el symlink o pasa enc_port:=/dev/ttyACM0
    enc_port = LaunchConfiguration('enc_port')
    declare_port = DeclareLaunchArgument('enc_port', default_value='/dev/rover_enc')

    # ── 1: base unica (cinematica + encoders -> /wheel/odometry) ──
    # OJO: para que rover_base.yaml aplique, su clave de nivel superior debe ser
    # 'rover_base:' (o '/**:'). Si sigue como 'skid_steer_base:', el nodo usa sus
    # defaults internos, que ya son tus valores validados (R=0.10, via=0.845, etc.).
    base = Node(
        package='circ_rover_hardware',
        executable='rover_base_node',
        name='rover_base',
        parameters=[base_yaml, {'port': enc_port}],
        output='screen',
    )

    # ── 2: EKF local (carga ekf_local.yaml; el nombre debe ser 'ekf_local') ──
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        parameters=[ekf_yaml],
        output='screen',
    )

    # ── (Opcional) bridge de IMU: descomentar cuando vuelva la IMU ──
    # imu = Node(
    #     package='circ_rover_hardware',
    #     executable='stm32_imu_bridge',
    #     name='imu_bridge',
    #     parameters=[{'port': '/dev/rover_imu', 'baud': 460800}],
    #     output='screen',
    # )

    return LaunchDescription([
        declare_port,
        base,
        ekf,
        # imu,
    ])