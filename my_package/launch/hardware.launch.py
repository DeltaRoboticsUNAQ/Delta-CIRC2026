"""
Launch file para control con hardware real (STM32).

Nodos lanzados:
  1. robot_state_publisher  — publica /robot_description y TF del modelo URDF
  2. stm32_hardware_bridge  — puente bidireccional ROS 2 ↔ STM32 (serie)
  3. rviz2 (opcional)       — visualización del estado real del brazo

Uso básico:
  ros2 launch my_package hardware.launch.py

Con parámetros:
  ros2 launch my_package hardware.launch.py \\
      port:=/dev/ttyUSB0 baudrate:=115200 sync_to_sim:=true use_rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_package')

    xacro_file = os.path.join(pkg_dir, 'urdf', 'arm.xacro')
    rviz_config_file = os.path.join(pkg_dir, 'config', 'arm.rviz')

    if not os.path.exists(rviz_config_file):
        import warnings
        warnings.warn(
            f'Archivo de configuración RViz no encontrado: {rviz_config_file}. '
            'RViz se abrirá con configuración por defecto.',
            UserWarning,
            stacklevel=2,
        )

    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # ── Argumentos de launch ──────────────────────────────────────────────────
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Puerto serie del STM32 (ej: /dev/ttyACM0, /dev/ttyUSB0)',
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate de comunicación con el STM32',
    )
    sync_to_sim_arg = DeclareLaunchArgument(
        'sync_to_sim',
        default_value='true',
        description=(
            'Si es true, publica el feedback del hardware en /joint_states '
            'para que la simulación refleje el estado real del brazo'
        ),
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Abrir RViz para visualizar el estado del brazo',
    )
    feedback_rate_arg = DeclareLaunchArgument(
        'feedback_rate_hz',
        default_value='50.0',
        description='Frecuencia de publicación del feedback del hardware (Hz)',
    )
    base_ticks_arg = DeclareLaunchArgument(
        'base_ticks_per_rev',
        default_value='2000',
        description='Ticks del encoder de la base por revolución completa',
    )
    wrist_ticks_arg = DeclareLaunchArgument(
        'wrist_ticks_per_rev',
        default_value='2000',
        description='Ticks de los encoders de la muñeca por revolución completa',
    )
    pot_min_arg = DeclareLaunchArgument(
        'pot_min',
        default_value='200',
        description='Valor ADC del potenciómetro en posición mínima del actuador',
    )
    pot_max_arg = DeclareLaunchArgument(
        'pot_max',
        default_value='3800',
        description='Valor ADC del potenciómetro en posición máxima del actuador',
    )

    # ── Nodos ─────────────────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    stm32_bridge = Node(
        package='my_package',
        executable='stm32_hardware_bridge.py',
        name='stm32_hardware_bridge',
        output='screen',
        parameters=[{
            'port':               LaunchConfiguration('port'),
            'baudrate':           LaunchConfiguration('baudrate'),
            'sync_to_sim':        LaunchConfiguration('sync_to_sim'),
            'feedback_rate_hz':   LaunchConfiguration('feedback_rate_hz'),
            'base_ticks_per_rev': LaunchConfiguration('base_ticks_per_rev'),
            'wrist_ticks_per_rev': LaunchConfiguration('wrist_ticks_per_rev'),
            'pot_min':            LaunchConfiguration('pot_min'),
            'pot_max':            LaunchConfiguration('pot_max'),
        }],
    )

    moveit_hw_bridge = Node(
        package='my_package',
        executable='moveit_hw_bridge.py',
        name='moveit_hw_bridge',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        sync_to_sim_arg,
        use_rviz_arg,
        feedback_rate_arg,
        base_ticks_arg,
        wrist_ticks_arg,
        pot_min_arg,
        pot_max_arg,
        robot_state_publisher,
        stm32_bridge,
        moveit_hw_bridge,
        rviz_node,
    ])
