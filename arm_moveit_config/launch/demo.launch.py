# src/arm_moveit_config/launch/demo.launch.py

import os
from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("arm_v3", package_name="arm_moveit_config")
        .to_moveit_configs()
    )

    # ── 1. Robot State Publisher ─────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # ── 2. ros2_control node ─────────────────────────────────────────────────
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            PathJoinSubstitution([
                FindPackageShare("arm_moveit_config"),
                "config", "ros2_controllers.yaml",
            ]),
        ],
    )

    # ── 3. Spawners con --inactive flag y timeout largo ──────────────────────
    #    Usamos TimerAction para esperar a que controller_manager esté listo
    joint_state_broadcaster_spawner = TimerAction(
        period=1.5,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", "30",
            ],
            output="screen",
        )]
    )

    arm_controller_spawner = TimerAction(
        period=2.5,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "arm_controller",
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", "30",
            ],
            output="screen",
        )]
    )

    # ── 4. move_group ────────────────────────────────────────────────────────
    move_group_node = TimerAction(
        period=4.0,
        actions=[Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": False},
                {"publish_robot_description_semantic": True},
            ],
        )]
    )

    # ── 5. RViz ──────────────────────────────────────────────────────────────
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("arm_moveit_config"),
        "config", "moveit.rviz",
    ])

    rviz_node = TimerAction(
        period=5.0,
        actions=[Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
            ],
        )]
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
        rviz_node,
    ])