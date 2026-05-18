import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    pkg_share = get_package_share_directory('circ_rover_description')
    
    # Ensure Gazebo can find the meshes by adding the workspace install/share to the GAZEBO_MODEL_PATH
    workspace_share_dir = os.path.join(pkg_share, '..', '..', 'share')
    set_gazebo_model_path_cmd = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[os.environ.get('GAZEBO_MODEL_PATH', ''), ':', workspace_share_dir]
    )

    # Process Xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'rover.xacro')
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    # Start robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Start joint_state_publisher to publish default joint states for RViz
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'source_list': ['gazebo_joint_states']
        }]
    )

    # Launch Gazebo Classic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'circ_rover',
                   '-z', '0.2'], # spawn slightly above ground
        output='screen'
    )

    # Launch RViz
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'rover.rviz')
    
    # Check if rviz config exists, else run without config argument
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    )

    return LaunchDescription([
        set_gazebo_model_path_cmd,
        node_joint_state_publisher,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        rviz_node
    ])
