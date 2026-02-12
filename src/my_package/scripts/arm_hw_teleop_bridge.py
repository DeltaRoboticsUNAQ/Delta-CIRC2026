#!/usr/bin/env python3
import sys
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import math


def getch_blocking():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class ArmHwTeleopBridge(Node):
    def __init__(self):
        super().__init__('arm_hw_teleop_bridge')

        # Parámetros de serial
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
            self.get_logger().info(f'Conectado a Arduino en {port} @ {baud} bps')
        except serial.SerialException as e:
            self.get_logger().error(f'No se pudo abrir el puerto {port}: {e}')
            self.ser = None

        # Publisher de joint_states (para RViz)
        self.pub_js = self.create_publisher(JointState, 'joint_states', 10)

        # Juntas lógicas (solo para visualización)
        self.joint_names = [
            'bracket_joint',       # base
            'humerus_low_joint',   # actuador 1
            'forearm_low_joint',   # actuador 2
            'ubracket_joint',      # muñeca pitch
            'endeffector_joint',   # muñeca roll
        ]
        self.positions = [0.0] * len(self.joint_names)

        # Límites (rad)
        limit_80deg = 80.0 * math.pi / 180.0
        limit_45deg = 45.0 * math.pi / 180.0
        self.limits = {
            'bracket_joint':      (-limit_80deg, limit_80deg),
            'humerus_low_joint':  (-1.0, 1.0),      # AJUSTA según el recorrido real del actuador 1
            'forearm_low_joint':  (-1.0, 1.0),      # AJUSTA según el recorrido real del actuador 2
            'ubracket_joint':     (-limit_45deg, limit_45deg),
            'endeffector_joint':  (-math.pi, math.pi),  # por ahora ±180°
        }

        # Timer para publicar estados
        self.timer = self.create_timer(0.05, self.timer_cb)

        # Hilo para leer teclado
        self._stop = False
        self._thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self._thread.start()

        self.get_logger().info('Teleop HW listo. Teclas iguales a las del Arduino.')
        self.print_help()

    def print_help(self):
        self.get_logger().info(
            "Teclas (en esta terminal):\n"
            "  A/D - Base izq/der\n"
            "  W/S - Muñeca subir/bajar\n"
            "  Q/E - Muñeca rotar izq/der\n"
            "  O/L - Actuador 1 extender/retraer\n"
            "  I/K - Actuador 2 extender/retraer\n"
            "  ESPACIO - Detener todo / reset visual"
        )

    def keyboard_loop(self):
        try:
            while not self._stop and rclpy.ok():
                key = getch_blocking()
                if key == '\x03':  # Ctrl+C
                    self._stop = True
                    break
                self.handle_key(key)
        except Exception as e:
            self.get_logger().error(f'Error en keyboard_loop: {e}')

    def handle_key(self, key: str):
        # Enviar tecla al Arduino tal cual
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.write(key.encode('ascii'))
            except serial.SerialException as e:
                self.get_logger().error(f'Error escribiendo en serie: {e}')

        # Actualizar modelo interno (solo RViz)
        step_base  = 0.05   # ~3°
        step_act   = 0.02
        step_wrist = 0.05

        # Índices por legibilidad
        IDX_BASE   = 0
        IDX_ACT1   = 1
        IDX_ACT2   = 2
        IDX_WPITCH = 3
        IDX_WROLL  = 4

        if key in ('a', 'A'):
            self.positions[IDX_BASE] -= step_base
        elif key in ('d', 'D'):
            self.positions[IDX_BASE] += step_base
        elif key in ('w', 'W'):
            self.positions[IDX_WPITCH] += step_wrist
        elif key in ('s', 'S'):
            self.positions[IDX_WPITCH] -= step_wrist
        elif key in ('q', 'Q'):
            self.positions[IDX_WROLL] -= step_wrist
        elif key in ('e', 'E'):
            self.positions[IDX_WROLL] += step_wrist
        elif key in ('o', 'O'):
            self.positions[IDX_ACT1] += step_act
        elif key in ('l', 'L'):
            self.positions[IDX_ACT1] -= step_act
        elif key in ('i', 'I'):
            self.positions[IDX_ACT2] += step_act
        elif key in ('k', 'K'):
            self.positions[IDX_ACT2] -= step_act
        elif key == ' ':
            for i in range(len(self.positions)):
                self.positions[i] = 0.0

        # Aplicar límites
        self.clamp_positions()

    def clamp_positions(self):
        for i, name in enumerate(self.joint_names):
            mn, mx = self.limits[name]
            v = self.positions[i]
            if v < mn:
                v = mn
            if v > mx:
                v = mx
            self.positions[i] = v

    def timer_cb(self):
        # Publicar JointState aproximado para RViz
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = list(self.positions)
        self.pub_js.publish(msg)

    def destroy_node(self):
        self._stop = True
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmHwTeleopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()