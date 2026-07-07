#!/usr/bin/env python3
"""
wheel_bridge.py   ──  paquete: circ_rover_hardware

Bridge serie para el ÚNICO STM32 que lee AMBOS encoders y controla AMBOS lados.
Un solo puerto, un solo nodo. (La IMU y el GPS tienen sus propios puertos/nodos.)

  STM32 -> ROS (CSV, 9 campos):  pos_L,vel_L,pos_R,vel_R,t,sp_L,sp_R,u_L,u_R
  ROS -> STM32 (setpoints):      SP:<rpm_izq>,<rpm_der>\n

velL = idx 1, velR = idx 3 (en RPM). Esta capa solo traduce serie <-> ROS;
la cinemática vive en skid_steer_base. Publica wheel_vel/left y wheel_vel/right
y se suscribe a cmd_wheel/left y right.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
import serial
import threading
import time


class WheelBridge(Node):
    def __init__(self):
        super().__init__('wheel_bridge')

        # ── Parámetros ──
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('send_rate', 50.0)        # [Hz] envío de setpoints
        self.declare_parameter('publish_debug', True)

        self.port      = self.get_parameter('port').value
        self.baud      = int(self.get_parameter('baud').value)
        self.send_rate = float(self.get_parameter('send_rate').value)
        self.debug     = bool(self.get_parameter('publish_debug').value)

        # ── Publishers: velocidad medida por lado ──
        self.vel_l_pub = self.create_publisher(Float64, 'wheel_vel/left', 10)
        self.vel_r_pub = self.create_publisher(Float64, 'wheel_vel/right', 10)
        if self.debug:
            self.dbg_pub = self.create_publisher(Float64MultiArray, 'wheel_debug', 10)

        # ── Subscribers: setpoints por lado (los calcula skid_steer_base) ──
        self.sp_l = 0.0
        self.sp_r = 0.0
        self.create_subscription(Float64, 'cmd_wheel/left',
                                 lambda m: setattr(self, 'sp_l', m.data), 10)
        self.create_subscription(Float64, 'cmd_wheel/right',
                                 lambda m: setattr(self, 'sp_r', m.data), 10)

        # ── Serie con reconexión ──
        self.ser = None
        self._open_serial()

        # Hilo de lectura + timer de escritura
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()
        self.create_timer(1.0 / self.send_rate, self.send_setpoints)

        self.get_logger().info(f'wheel_bridge (2 lados) en {self.port} @ {self.baud}')

    def _open_serial(self):
        while rclpy.ok():
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                return
            except serial.SerialException as e:
                self.get_logger().warn(f'no abre {self.port}: {e} — reintento en 1s')
                time.sleep(1.0)

    def read_loop(self):
        """Lee CSV de 9 campos: pos_L,vel_L,pos_R,vel_R,t,sp_L,sp_R,u_L,u_R
           (velL = idx1, velR = idx3, en RPM)."""
        while rclpy.ok():
            try:
                raw = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not raw:
                    continue
                parts = raw.split(',')
                if len(parts) != 9:
                    continue
                vals = [float(p) for p in parts]

                ml = Float64(); ml.data = vals[1]; self.vel_l_pub.publish(ml)
                mr = Float64(); mr.data = vals[3]; self.vel_r_pub.publish(mr)

                if self.debug:
                    d = Float64MultiArray(); d.data = vals; self.dbg_pub.publish(d)

            except (ValueError, UnicodeDecodeError):
                continue
            except serial.SerialException as e:
                self.get_logger().warn(f'serie caída: {e} — reabriendo')
                self._open_serial()
            except Exception as e:
                self.get_logger().warn(f'error: {e}')

    def send_setpoints(self):
        """Manda ambos setpoints en un solo comando: SP:L,R."""
        try:
            self.ser.write(f'SP:{self.sp_l:.2f},{self.sp_r:.2f}\n'.encode('utf-8'))
        except serial.SerialException:
            pass  # la reconexión la maneja read_loop


def main(args=None):
    rclpy.init(args=args)
    node = WheelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None:
            node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()