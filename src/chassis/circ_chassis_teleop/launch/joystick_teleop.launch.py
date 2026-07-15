import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('circ_chassis_teleop')

    joy_params = os.path.join(share, 'config', 'joy_params.yaml')
    default_cfg = os.path.join(share, 'config', 'xbox_default.config.yaml')

    config_filepath = LaunchConfiguration('config_filepath')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    return LaunchDescription(
        [
            DeclareLaunchArgument('config_filepath', default_value=default_cfg),
            DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_teleop'),

            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                output='screen',
                parameters=[joy_params],
            ),
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_twist_joy_node',
                output='screen',
                parameters=[config_filepath, {'publish_stamped_twist': False}],
                remappings=[('/cmd_vel', cmd_vel_topic)],
                
            ),
        ]
    )
