import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_name = 'my_package'
    pkg_dir = get_package_share_directory(package_name)

    # Gazebo resolves model://<name>/... by searching GAZEBO_MODEL_PATH for a folder named <name>.
    # We add the parent share directory so model://my_package/... resolves to <share>/my_package/...
    share_dir = os.path.dirname(pkg_dir)
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = f'{share_dir}:{gazebo_model_path}' if gazebo_model_path else share_dir
    set_gazebo_model_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=gazebo_model_path)

    # Tu xacro
    xacro_file = os.path.join(pkg_dir, 'urdf', 'arm.xacro')

    # World por defecto
    default_world = os.path.join(pkg_dir, 'worlds', 'empty.world')

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
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'paused': LaunchConfiguration('paused'),
        }.items(),
    )

    # robot_state_publisher para TF
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description,
        }],
    )

    description_publisher = Node(
        package=package_name,
        executable='robot_description_publisher.py',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'topic': LaunchConfiguration('robot_description_topic'),
            'publish_period_s': 0.5,
        }],
    )

    # Nodo de spawn en Gazebo
    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_arm',
            '-topic', LaunchConfiguration('robot_description_topic'),
            '-timeout', '60',
            # Helps Gazebo resolve package:// URIs in meshes
            '-package_to_model',
            # Spawn slightly above ground
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
            '-R', LaunchConfiguration('spawn_roll'),
            '-P', LaunchConfiguration('spawn_pitch'),
            '-Y', LaunchConfiguration('spawn_yaw'),
        ],
        output='screen',
    )

    delayed_spawn = TimerAction(period=2.0, actions=[spawn_node])

    set_initial_joints = Node(
        package=package_name,
        executable='gazebo_set_initial_joint_positions.py',
        output='screen',
        parameters=[{
            'model_name': 'my_arm',
            'joint_names': [
                'bracket_joint',
                'humerus_low_joint',
                'forearm_low_joint',
                'ubracket_joint',
                'endeffector_joint',
            ],
            'joint_positions': [
                LaunchConfiguration('bracket_joint_pos'),
                LaunchConfiguration('humerus_low_joint_pos'),
                LaunchConfiguration('forearm_low_joint_pos'),
                LaunchConfiguration('ubracket_joint_pos'),
                LaunchConfiguration('endeffector_joint_pos'),
            ],
        }],
    )

    delayed_set_initial_joints = TimerAction(period=4.0, actions=[set_initial_joints])

    return LaunchDescription([
        set_gazebo_model_path,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock (Gazebo) if true',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Path to a Gazebo world file',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Run Gazebo client (gzclient) if true',
        ),
        DeclareLaunchArgument(
            'paused',
            default_value='false',
            description='Start Gazebo paused if true',
        ),
        DeclareLaunchArgument('spawn_x', default_value='0.0', description='Initial spawn X (m)'),
        DeclareLaunchArgument('spawn_y', default_value='0.0', description='Initial spawn Y (m)'),
        DeclareLaunchArgument('spawn_z', default_value='0.1', description='Initial spawn Z (m)'),
        DeclareLaunchArgument('spawn_roll', default_value='0.0', description='Initial roll (rad)'),
        DeclareLaunchArgument('spawn_pitch', default_value='0.0', description='Initial pitch (rad)'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='Initial yaw (rad)'),
        DeclareLaunchArgument(
            'robot_description_topic',
            default_value='robot_description',
            description='Topic to publish the URDF as std_msgs/String for Gazebo spawning',
        ),
        DeclareLaunchArgument('bracket_joint_pos', default_value='0.0', description='Initial bracket_joint position (rad)'),
        DeclareLaunchArgument('humerus_low_joint_pos', default_value='0.0', description='Initial humerus_low_joint position (rad)'),
        DeclareLaunchArgument('forearm_low_joint_pos', default_value='0.0', description='Initial forearm_low_joint position (rad)'),
        DeclareLaunchArgument('ubracket_joint_pos', default_value='0.0', description='Initial ubracket_joint position (rad)'),
        DeclareLaunchArgument('endeffector_joint_pos', default_value='0.0', description='Initial endeffector_joint position (rad)'),
        gazebo_launch,
        rsp_node,
        description_publisher,
        delayed_spawn,
        delayed_set_initial_joints,
    ])