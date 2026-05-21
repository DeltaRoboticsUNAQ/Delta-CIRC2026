import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Obtener la ruta del paquete
    package_dir = get_package_share_directory('my_package')
    
    # Rutas de archivos
    xacro_file = os.path.join(package_dir, 'urdf', 'arm.xacro')
    rviz_config_file = os.path.join(package_dir, 'config', 'arm.rviz')
    
    # Procesar el archivo xacro y convertirlo a URDF
    robot_description_content = Command([
        'xacro ',
        xacro_file
    ])
    
    robot_description = ParameterValue(
        robot_description_content,
        value_type=str
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Node para procesar el archivo xacro y publicar el estado del robot
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_description,
            }],
        ),
        
        # Node para publicar las posiciones de las juntas (por defecto en cero)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])


