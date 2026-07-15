#!/usr/bin/env python3
"""
MoveIt → Hardware Bridge

Implementa el action server FollowJointTrajectory que MoveIt necesita
para ejecutar trayectorias en el hardware real (STM32).

Flujo:
  MoveIt → /arm_controller/follow_joint_trajectory (action)
         → moveit_hw_bridge
         → /joint_states_cmd (JointState)
         → stm32_hardware_bridge
         → STM32

Uso (junto con hardware.launch.py):
  ros2 launch my_package hardware.launch.py port:=/dev/ttyACM0
  # En otra terminal, lanzar MoveIt apuntando al hardware real:
  ros2 launch arm_moveit_config move_group.launch.py
"""

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState


# Tolerancia de posición para considerar un punto como alcanzado (rad)
# Solo se usa si wait_for_feedback=true
POSITION_TOLERANCE = 0.05


class MoveItHwBridge(Node):
    """
    Action server FollowJointTrajectory que traduce trayectorias de MoveIt
    a comandos JointState para el stm32_hardware_bridge.
    """

    def __init__(self):
        super().__init__('moveit_hw_bridge')

        self.declare_parameter('action_name', 'arm_controller/follow_joint_trajectory')
        self.declare_parameter('cmd_topic',   '/joint_states_cmd')
        self.declare_parameter('fb_topic',    '/hardware/joint_states')
        # Si es true, espera confirmación de posición antes de pasar al siguiente punto.
        # Si es false, ejecuta la trayectoria por tiempo (más predecible).
        self.declare_parameter('wait_for_feedback', False)

        action_name      = self.get_parameter('action_name').get_parameter_value().string_value
        cmd_topic        = self.get_parameter('cmd_topic').get_parameter_value().string_value
        fb_topic         = self.get_parameter('fb_topic').get_parameter_value().string_value
        self.wait_for_fb = self.get_parameter('wait_for_feedback').get_parameter_value().bool_value

        # Publisher de comandos hacia el bridge del STM32
        self.pub_cmd = self.create_publisher(JointState, cmd_topic, 10)

        # Subscriber de feedback del hardware (posiciones reales)
        self._hw_positions: dict[str, float] = {}
        self._hw_lock = threading.Lock()
        self.create_subscription(JointState, fb_topic, self._fb_callback, 10)

        # Action server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self._cancel_flag = False

        self.get_logger().info(
            f'MoveIt HW Bridge iniciado\n'
            f'  Action  : /{action_name}\n'
            f'  Publica : {cmd_topic}\n'
            f'  Feedback: {fb_topic}\n'
            f'  Modo    : {"esperar feedback" if self.wait_for_fb else "por tiempo"}'
        )

    # ── Callbacks del action server ───────────────────────────────────────────

    def _goal_callback(self, goal_request):
        self.get_logger().info('MoveIt envió nueva trayectoria — aceptando.')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Cancelación solicitada.')
        self._cancel_flag = True
        return CancelResponse.ACCEPT

    async def _execute_callback(self, goal_handle):
        self._cancel_flag = False
        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names
        points = trajectory.points

        self.get_logger().info(
            f'Ejecutando trayectoria: {len(points)} puntos, '
            f'joints: {joint_names}'
        )

        feedback_msg = FollowJointTrajectory.Feedback()
        feedback_msg.joint_names = joint_names

        start_time = self.get_clock().now()

        for i, point in enumerate(points):
            if self._cancel_flag or not goal_handle.is_active:
                self.get_logger().info('Trayectoria cancelada.')
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            # Publicar el punto como comando de posición
            cmd = JointState()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.name = list(joint_names)
            cmd.position = list(point.positions)
            cmd.velocity = list(point.velocities) if point.velocities else []
            cmd.effort   = []
            self.pub_cmd.publish(cmd)

            self.get_logger().debug(
                f'Punto {i+1}/{len(points)}: '
                + ', '.join(f'{n}={p:.3f}' for n, p in zip(joint_names, point.positions))
            )

            # Enviar feedback a MoveIt
            feedback_msg.desired.positions  = list(point.positions)
            feedback_msg.desired.velocities = list(point.velocities) if point.velocities else []
            with self._hw_lock:
                feedback_msg.actual.positions = [
                    self._hw_positions.get(n, p)
                    for n, p in zip(joint_names, point.positions)
                ]
            feedback_msg.error.positions = [
                d - a for d, a in zip(feedback_msg.desired.positions, feedback_msg.actual.positions)
            ]
            goal_handle.publish_feedback(feedback_msg)

            # Esperar hasta el tiempo del siguiente punto (o feedback de hardware)
            if i + 1 < len(points):
                next_point_time = points[i + 1].time_from_start
                current_time = self.get_clock().now() - start_time
                wait_secs = (
                    next_point_time.sec + next_point_time.nanosec * 1e-9
                    - current_time.nanoseconds * 1e-9
                )
                if self.wait_for_fb:
                    self._wait_for_position(joint_names, point.positions, timeout=max(wait_secs, 0.5))
                elif wait_secs > 0:
                    time.sleep(wait_secs)

        # Último punto: esperar un poco para que el hardware llegue
        last = points[-1]
        last_wait = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9
        elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9
        remaining = last_wait - elapsed + 0.2   # +0.2s de margen
        if remaining > 0:
            if self.wait_for_fb:
                self._wait_for_position(joint_names, last.positions, timeout=max(remaining, 1.0))
            else:
                time.sleep(remaining)

        self.get_logger().info('Trayectoria completada.')
        goal_handle.succeed()

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    # ── Feedback del hardware ─────────────────────────────────────────────────

    def _fb_callback(self, msg: JointState):
        with self._hw_lock:
            for name, pos in zip(msg.name, msg.position):
                self._hw_positions[name] = pos

    def _wait_for_position(self, joint_names, target_positions, timeout: float):
        """Bloquea hasta que el hardware alcanza la posición objetivo o timeout."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._hw_lock:
                errors = [
                    abs(self._hw_positions.get(n, 1e9) - p)
                    for n, p in zip(joint_names, target_positions)
                ]
            if all(e < POSITION_TOLERANCE for e in errors):
                return
            time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    node = MoveItHwBridge()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
