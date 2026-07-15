#!/usr/bin/env python3
# =====================================================================
# Nav2 bring-up  -  circ2025  (GPS outdoor, sin mapa estatico)
#
# Arranca: controller, planner, behaviors, bt_navigator, waypoint_follower
# y el lifecycle_manager.  NO arranca map_server ni amcl (el map->odom
# lo da tu ekf_global/navsat).
#
# La salida cmd_vel del controller y del behavior server se remapea a
# /cmd_vel_raw  ->  tu UnifiedSafetyController  ->  /cmd_vel  ->  motores.
#
# Requiere que YA esten corriendo: EKFs (map->odom->base_link),
# drivers de sensores, la OAK (/scan) y el UnifiedSafetyController.
# =====================================================================
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

PKG = 'circ_rover_navigation'   # <-- CAMBIA si tu paquete se llama distinto


def generate_launch_description():
    pkg_share = get_package_share_directory(PKG)
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='true para Gazebo, false en el robot real')
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params)
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true')

    # Inyecta use_sim_time en todos los nodos del yaml
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True,
    )

    # Nodos a gestionar por el lifecycle manager
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    # --- REMAP CLAVE: salida de velocidad -> /cmd_vel_raw ---
    cmd_vel_remap = [('cmd_vel', '/cmd_vel_raw')]

    controller = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen',
        parameters=[configured_params],
        remappings=cmd_vel_remap,   # controller -> /cmd_vel_raw
    )

    planner = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen',
        parameters=[configured_params],
    )

    behaviors = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen',
        parameters=[configured_params],
        remappings=cmd_vel_remap,   # spins/backups de recuperacion -> /cmd_vel_raw
    )

    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen',
        parameters=[configured_params],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower',
        name='waypoint_follower', output='screen',
        parameters=[configured_params],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params,
        declare_autostart,
        controller,
        planner,
        behaviors,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
    ])