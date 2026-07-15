# src/arm_tasks/arm_tasks/arm_motion_client.py
"""
Cliente reutilizable de MoveIt 2 para todas las tasks de competencia.
Encapsula la comunicación con move_group y el arm_controller.
Adaptado para brazo arm_v3 con 5 joints.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from builtin_interfaces.msg import Duration as RosDuration

import math
import time


# Nombres exactos de tus joints (del URDF)
JOINT_NAMES = [
    'bracket_joint',       # base - motor DC + encoder
    'humerus_low_joint',   # hombro - actuador lineal + potenciómetro
    'forearm_low_joint',   # codo - actuador lineal + potenciómetro
    'ubracket_joint',      # muñeca pitch - diferencial
    'endeffector_joint',   # muñeca roll - diferencial
]

# Límites del URDF (en radianes)
JOINT_LIMITS = {
    'bracket_joint':       (-1.5,  1.5),
    'humerus_low_joint':   (-1.5,  1.5),
    'forearm_low_joint':   (-1.5,  1.5),
    'ubracket_joint':      (-0.8,  0.8),
    'endeffector_joint':   (-4.5,  4.5),
}

# Poses predefinidas para competencia (en radianes)
NAMED_POSES = {
    'home':        [0.0,   0.0,   0.0,   0.0,  0.0],
    'pre_press':   [0.0,  -0.8,   0.8,   0.0,  0.0],  # brazo extendido
    'pick_low':    [0.0,  -0.3,   0.5,   0.0,  0.0],  # pick a 0.4m
    'pick_mid':    [0.0,  -0.5,   0.6,   0.0,  0.0],  # pick a 0.7m
    'pick_high':   [0.0,  -0.8,   0.9,   0.0,  0.0],  # pick a 1.0m
    'pass_height': [0.0,  -0.6,   0.7,   0.0,  0.0],  # entrega a 0.5m
    'stow':        [0.0,   0.5,  -0.3,   0.0,  0.0],  # guardado compacto
}


class ArmMotionClient:
    """
    Cliente de alto nivel para controlar el brazo arm_v3.
    Usa directamente el action /arm_controller/follow_joint_trajectory.
    Compatible con mock_components (simulación) y hardware real.
    """

    def __init__(self, node: Node):
        self.node = node
        self.current_joints = [0.0] * 5

        # Action client al JointTrajectoryController
        self._action_client = ActionClient(
            node,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        # Suscribirse a joint_states para saber posición actual
        self._js_sub = node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10
        )

        self.node.get_logger().info(
            "ArmMotionClient inicializado — "
            "esperando /arm_controller/follow_joint_trajectory..."
        )

        # Esperar a que el action server esté disponible
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(
                "Action server no disponible. "
                "¿Está corriendo demo.launch.py?"
            )

    def _joint_state_cb(self, msg: JointState):
        """Actualiza la posición actual de los joints."""
        for i, name in enumerate(JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self.current_joints[i] = msg.position[idx]

    def go_to_named_pose(self, pose_name: str,
                          duration_sec: float = 2.0) -> bool:
        """
        Mueve el brazo a una pose predefinida por nombre.

        Args:
            pose_name: 'home', 'pre_press', 'pick_low', etc.
            duration_sec: tiempo para completar el movimiento

        Returns:
            True si llegó exitosamente
        """
        if pose_name not in NAMED_POSES:
            self.node.get_logger().error(
                f"Pose '{pose_name}' no existe. "
                f"Disponibles: {list(NAMED_POSES.keys())}"
            )
            return False

        positions = NAMED_POSES[pose_name]
        self.node.get_logger().info(
            f"Moviendo a pose '{pose_name}': {positions}"
        )
        return self.go_to_joint_positions(positions, duration_sec)

    def go_to_joint_positions(self, positions: list,
                               duration_sec: float = 2.0) -> bool:
        """
        Mueve el brazo a posiciones de joint específicas.

        Args:
            positions: lista de 5 valores en radianes
                       [bracket, humerus, forearm, ubracket, endeffector]
            duration_sec: tiempo para completar el movimiento

        Returns:
            True si llegó exitosamente
        """
        if len(positions) != 5:
            self.node.get_logger().error(
                f"Se esperan 5 posiciones, se recibieron {len(positions)}"
            )
            return False

        # Verificar límites antes de enviar
        if not self._check_limits(positions):
            return False

        # Construir mensaje de trayectoria
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.velocities = [0.0] * 5
        point.time_from_start = RosDuration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        goal.trajectory.points = [point]

        # Enviar goal y esperar resultado
        return self._send_goal_and_wait(goal, timeout_sec=duration_sec + 3.0)

    def go_to_joint_positions_smooth(self, waypoints: list,
                                      total_duration: float = 3.0) -> bool:
        """
        Ejecuta una trayectoria suave con múltiples waypoints.
        Útil para movimientos precisos como presionar botones.

        Args:
            waypoints: lista de listas [[p1,p2,p3,p4,p5], ...]
            total_duration: duración total de la trayectoria

        Returns:
            True si completó exitosamente
        """
        if not waypoints:
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = JOINT_NAMES

        dt = total_duration / len(waypoints)

        for i, positions in enumerate(waypoints):
            if not self._check_limits(positions):
                return False
            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in positions]
            point.velocities = [0.0] * 5
            t = dt * (i + 1)
            point.time_from_start = RosDuration(
                sec=int(t),
                nanosec=int((t % 1) * 1e9)
            )
            goal.trajectory.points.append(point)

        return self._send_goal_and_wait(
            goal, timeout_sec=total_duration + 3.0
        )

    def rotate_base(self, angle_rad: float,
                    duration_sec: float = 2.0) -> bool:
        """
        Rota solo la base (bracket_joint — motor DC + encoder).
        Los demás joints mantienen su posición actual.
        """
        target = list(self.current_joints)
        target[0] = angle_rad
        self.node.get_logger().info(
            f"Rotando base a {math.degrees(angle_rad):.1f}°"
        )
        return self.go_to_joint_positions(target, duration_sec)

    def press_button(self, approach_positions: list,
                     press_depth_j3: float = 0.05) -> bool:
        """
        Secuencia específica para presionar botones pequeños (1"x1").
        Mueve al approach, luego hace el press con movimiento de
        forearm_low_joint (codo).

        Args:
            approach_positions: posición del brazo frente al botón
            press_depth_j3: cuánto avanzar en forearm_low_joint
                           para presionar (radianes)
        """
        self.node.get_logger().info("Iniciando secuencia press_button")

        # 1. Ir a posición de approach
        if not self.go_to_joint_positions(approach_positions, 2.0):
            return False

        # 2. Avanzar para presionar (aumentar forearm)
        press_positions = list(approach_positions)
        press_positions[2] += press_depth_j3  # forearm_low_joint

        if not self._check_limits(press_positions):
            self.node.get_logger().warn("Press excede límites, reduciendo")
            press_positions[2] = JOINT_LIMITS['forearm_low_joint'][1] * 0.9

        if not self.go_to_joint_positions(press_positions, 0.5):
            return False

        # 3. Mantener presionado 0.3 segundos
        time.sleep(0.3)

        # 4. Retractar
        if not self.go_to_joint_positions(approach_positions, 0.5):
            return False

        self.node.get_logger().info("press_button completado ✓")
        return True

    def wrist_differential_move(self, pitch_rad: float,
                                  roll_rad: float,
                                  duration_sec: float = 1.0) -> bool:
        """
        Mueve la muñeca diferencial.
        ubracket_joint  = pitch (arriba/abajo)
        endeffector_joint = roll (rotación)

        Args:
            pitch_rad: ángulo de pitch deseado
            roll_rad: ángulo de roll deseado
        """
        target = list(self.current_joints)
        target[3] = pitch_rad    # ubracket_joint
        target[4] = roll_rad     # endeffector_joint
        self.node.get_logger().info(
            f"Muñeca → pitch: {math.degrees(pitch_rad):.1f}°, "
            f"roll: {math.degrees(roll_rad):.1f}°"
        )
        return self.go_to_joint_positions(target, duration_sec)

    def go_home(self, duration_sec: float = 2.0) -> bool:
        """Vuelve a posición HOME (todos los joints en 0)."""
        self.node.get_logger().info("Volviendo a HOME")
        return self.go_to_named_pose('home', duration_sec)

    def get_current_joints(self) -> list:
        """Retorna las posiciones actuales de los joints."""
        return list(self.current_joints)

    # ── Métodos privados ──────────────────────────────────────────────────────

    def _check_limits(self, positions: list) -> bool:
        """Verifica que las posiciones estén dentro de los límites del URDF."""
        for i, (name, pos) in enumerate(zip(JOINT_NAMES, positions)):
            lo, hi = JOINT_LIMITS[name]
            if pos < lo or pos > hi:
                self.node.get_logger().error(
                    f"LÍMITE EXCEDIDO: {name} = {pos:.3f} rad "
                    f"(límites: [{lo}, {hi}])"
                )
                return False
        return True

    def _send_goal_and_wait(self, goal: FollowJointTrajectory.Goal,
                             timeout_sec: float = 10.0) -> bool:
        """Envía el goal al action server y espera el resultado."""
        if not self._action_client.server_is_ready():
            self.node.get_logger().error("Action server no disponible")
            return False

        # Enviar goal
        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self.node, send_future, timeout_sec=5.0
        )

        if not send_future.done():
            self.node.get_logger().error("Timeout enviando goal")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Goal rechazado por el controller")
            return False

        # Esperar resultado
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node, result_future, timeout_sec=timeout_sec
        )

        if not result_future.done():
            self.node.get_logger().error("Timeout esperando resultado")
            return False

        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.node.get_logger().info("✓ Movimiento completado")
            return True
        else:
            self.node.get_logger().error(
                f"✗ Error en ejecución: {result.error_string} "
                f"(código: {result.error_code})"
            )
            return False