import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_name = 'my_package'
    pkg_dir = get_package_share_directory(package_name)

    xacro_file = os.path.join(pkg_dir, 'urdf', 'arm.xacro')
    rviz_config_file = os.path.join(pkg_dir, 'config', 'arm.rviz')

    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        ),
    ]

    return LaunchDescription(nodes)