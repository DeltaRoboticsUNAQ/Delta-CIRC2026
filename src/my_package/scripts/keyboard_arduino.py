#!/usr/bin/env python3
import sys
import termios
import tty
import serial
import rclpy
from rclpy.node import Node


class KeyboardTeleopArduino(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_arduino')

        # Parámetros configurables desde launch o CLI
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 9600)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.get_logger().info(f'Abriendo puerto serie {port} @ {baud}...')
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)

        self.get_logger().info('Listo. Usa el teclado como antes:')
        self.get_logger().info('  A/D - Base izquierda/derecha')
        self.get_logger().info('  W/S - Muñeca subir/bajar')
        self.get_logger().info('  Q/E - Muñeca rotar izq/der')
        self.get_logger().info('  O/L - Actuador 1 extender/retraer')
        self.get_logger().info('  I/K - Actuador 2 extender/retraer')
        self.get_logger().info('  ESPACIO - Detener todo')
        self.get_logger().info('  Ctrl+C para salir')

    def send_key(self, key: str):
        if not key:
            return
        # Enviar el carácter tal cual al Arduino
        self.ser.write(key.encode('utf-8'))
        self.ser.flush()


def getch():
    """Lee una tecla sin esperar Enter (modo raw) en terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)  # lee 1 carácter
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopArduino()

    try:
        while rclpy.ok():
            key = getch()

            # Opcional: mostrar qué se lee
            if key == '\x03':  # Ctrl+C
                break

            node.get_logger().info(f'Tecla: {repr(key)}')
            node.send_key(key)

    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Saliendo...')
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()