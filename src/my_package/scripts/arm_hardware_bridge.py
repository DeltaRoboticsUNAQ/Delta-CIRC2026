#!/usr/bin/env python3
import math
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ArmHardwareBridge(Node):
    def __init__(self):
        super().__init__('arm_hardware_bridge')

        # Parámetros configurables
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Abrir puerto serie
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
            self.get_logger().info(f'Conectado a {port} @ {baud} bps')
        except serial.SerialException as e:
            self.get_logger().error(f'No se pudo abrir el puerto serie {port}: {e}')
            self.ser = None

        # Mapa de joints que nos interesan
        self.joint_index = {
            'bracket_joint': None,
            'humerus_low_joint': None,
            'forearm_low_joint': None,
            'ubracket_joint': None,
            'endeffector_joint': None,
        }

        self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_cb,
            10
        )

    def joint_states_cb(self, msg: JointState):
        if self.ser is None or not self.ser.is_open:
            return

        # Mapear nombres de joints a índices la primera vez
        if any(v is None for v in self.joint_index.values()):
            for i, name in enumerate(msg.name):
                if name in self.joint_index:
                    self.joint_index[name] = i

            if any(v is None for v in self.joint_index.values()):
                # Aún no conocemos todos los índices
                return

        try:
            b = msg.position[self.joint_index['bracket_joint']]
            h = msg.position[self.joint_index['humerus_low_joint']]
            f = msg.position[self.joint_index['forearm_low_joint']]
            u = msg.position[self.joint_index['ubracket_joint']]
            e = msg.position[self.joint_index['endeffector_joint']]
        except (IndexError, TypeError):
            return

        # De radianes a grados
        b_deg = math.degrees(b)
        h_deg = math.degrees(h)
        f_deg = math.degrees(f)
        u_deg = math.degrees(u)
        e_deg = math.degrees(e)

        # Reforzar límites lógicos por seguridad
        b_deg = max(-80.0, min(80.0, b_deg))
        u_deg = max(-80.0, min(80.0, u_deg))

        # Comando de texto: sigue al teclado en tiempo real
        cmd = f"B:{b_deg:.1f};H:{h_deg:.1f};F:{f_deg:.1f};U:{u_deg:.1f};E:{e_deg:.1f}\n"

        # Log para depurar
        self.get_logger().info(f'CMD -> Arduino: {cmd.strip()}')

        try:
            self.ser.write(cmd.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().error(f'Error escribiendo en serie: {e}')
            try:
                self.ser.close()
            except Exception:
                pass

    def destroy_node(self):
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmHardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()