from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    speed = LaunchConfiguration('speed')
    turn = LaunchConfiguration('turn')

    # ros2 CLI expects a single token: cmd_vel:=/some/topic
    remap = PythonExpression(["'cmd_vel:=' + '", cmd_vel_topic, "'"])

    return LaunchDescription(
        [
            DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_teleop'),
            DeclareLaunchArgument('speed', default_value='0.5'),
            DeclareLaunchArgument('turn', default_value='1.0'),

            ExecuteProcess(
                cmd=[
                    'ros2',
                    'run',
                    'teleop_twist_keyboard',
                    'teleop_twist_keyboard',
                    '--speed',
                    speed,
                    '--turn',
                    turn,
                    '--ros-args',
                    '-r',
                    remap,
                ],
                output='screen',
                emulate_tty=True,
            ),
        ]
    )
