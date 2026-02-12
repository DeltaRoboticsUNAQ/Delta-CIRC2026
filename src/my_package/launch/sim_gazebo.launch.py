import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_name = 'my_package'
    pkg_dir = get_package_share_directory(package_name)

    # Tu xacro
    xacro_file = os.path.join(pkg_dir, 'urdf', 'arm.xacro')

    # Generar robot_description desde xacro
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Launch de Gazebo vacío
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    # robot_state_publisher para TF
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Nodo de spawn en Gazebo
    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_arm',
            '-topic', 'robot_description',
            # Opcionalmente posición inicial:
            # '-x', '0', '-y', '0', '-z', '0.0'
        ],
        output='screen',
    )

    return LaunchDescription([
        gazebo_launch,
        rsp_node,
        spawn_node,
    ])