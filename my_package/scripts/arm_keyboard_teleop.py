#!/usr/bin/env python3
import sys
import termios
import tty
import threading
from typing import Dict

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def getch_blocking():
    """Lee una tecla de forma bloqueante en el terminal actual."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class ArmKeyboardTeleop(Node):
    def __init__(self):
        super().__init__('arm_keyboard_teleop')

        # Paso angular por tecla (rad)
        self.step = 0.05  # ~3 grados

        # SOLO las juntas principales que vamos a controlar
        self.joint_names = [
            'bracket_joint',
            'humerus_low_joint',
            'forearm_low_joint',
            'ubracket_joint',
            'endeffector_joint',
        ]

        # Estado actual
        self.positions: Dict[str, float] = {name: 0.0 for name in self.joint_names}

        # ======== EL CAMBIO MÁGICO PARA GAZEBO ========
        # Publisher de /arm_controller/joint_trajectory
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Timer para publicar a 50 Hz
        self.timer = self.create_timer(0.02, self.timer_cb)

        # Hilo separado para leer teclado
        self._stop = False
        self._thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self._thread.start()

        self.get_logger().info('Teleop por teclado listo y conectado a Gazebo.')
        self.print_help()

    def print_help(self):
        self.get_logger().info('========================================')
        self.get_logger().info('Controles (simulación Gazebo/RViz):')
        self.get_logger().info('  A/D : base bracket_joint')
        self.get_logger().info('  W/S : hombro humerus_low_joint')
        self.get_logger().info('  R/F : codo forearm_low_joint')
        self.get_logger().info('  T/G : muñeca pitch (ubracket_joint)')
        self.get_logger().info('  Y/H : muñeca roll (endeffector_joint)')
        self.get_logger().info('  ESPACIO : reset posiciones a 0')
        self.get_logger().info('  Ctrl+C (en esta terminal) : salir')
        self.get_logger().info('========================================')

    def timer_cb(self):
        """Publicar el JointTrajectory actual."""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [self.positions[n] for n in self.joint_names]
        # Tiempo para que el movimiento sea fluido en el simulador
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 50000000 
        
        msg.points.append(point)
        self.pub.publish(msg)

    # ------------------ Entrada por teclado ------------------ #

    def keyboard_loop(self):
        try:
            while not self._stop and rclpy.ok():
                key = getch_blocking()
                self.handle_key(key)
        except Exception as e:
            self.get_logger().error(f'Error en keyboard_loop: {e}')

    def handle_key(self, key: str):
        if key in ('\x03',):  # Ctrl+C
            self._stop = True
            return

        if key == ' ':
            for n in self.positions:
                self.positions[n] = 0.0
            self.get_logger().info('Reset de todas las articulaciones a 0')
            return

        # Rotacionales
        if key in ('a', 'A'):
            self.positions['bracket_joint'] -= self.step
        elif key in ('d', 'D'):
            self.positions['bracket_joint'] += self.step

        elif key in ('w', 'W'):
            self.positions['humerus_low_joint'] += self.step
        elif key in ('s', 'S'):
            self.positions['humerus_low_joint'] -= self.step

        elif key in ('r', 'R'):
            self.positions['forearm_low_joint'] += self.step
        elif key in ('f', 'F'):
            self.positions['forearm_low_joint'] -= self.step

        elif key in ('t', 'T'):
            self.positions['ubracket_joint'] += self.step
        elif key in ('g', 'G'):
            self.positions['ubracket_joint'] -= self.step

        elif key in ('y', 'Y'):
            self.positions['endeffector_joint'] += self.step
        elif key in ('h', 'H'):
            self.positions['endeffector_joint'] -= self.step

        self.clamp_limits()

    def clamp_limits(self):
        # 80 grados = 80 * pi / 180 ≈ 1.396 rad
        limit_80deg = 1.3962634

        lim = {
            'bracket_joint': (-limit_80deg, limit_80deg),
            'humerus_low_joint': (-1.5, 0.0),          # Ojo: ajusté a tus límites del xacro
            'forearm_low_joint': (-1.216, 0.0),          # Ojo: ajusté a tus límites del xacro
            'ubracket_joint': (-0.785, 0.785),  # muñeca pitch
            'endeffector_joint': (-3.1416, 3.1416),          # muñeca roll
        }
        for n, (mn, mx) in lim.items():
            v = self.positions[n]
            if v < mn:
                v = mn
            if v > mx:
                v = mx
            self.positions[n] = v

    def destroy_node(self):
        self._stop = True
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArmKeyboardTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Saliendo de teleop...')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()