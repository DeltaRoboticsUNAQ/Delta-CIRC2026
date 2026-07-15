#!/usr/bin/env python3
"""
rover_base_node.py   ──  paquete: circ_rover_hardware (o circ_rover_navigation)

Nodo ÚNICO que fusiona wheel_bridge + skid_steer_base para el rover skid-steer
de 6 ruedas (3 por lado, 1 encoder por lado, UN solo STM32 con SP:L,R).

Hace tres cosas, todas con las mismas constantes:

  1) SERIE -> ROS   : lee CSV de 9 campos del STM32 y saca las velocidades medidas.
        CSV: pos_L,vel_L,pos_R,vel_R,t,sp_L,sp_R,u_L,u_R   (velL=idx1, velR=idx3, RPM)
  2) CINEMATICA     : /cmd_vel (Twist) -> setpoints RPM por lado -> SP:<izq>,<der>\\n
  3) ODOMETRIA      : velocidades medidas -> /wheel/odometry (para el EKF)

Flujo:   /cmd_vel --> [cinematica] --> SP:L,R --serie--> STM32
         STM32 --serie(CSV)--> [vel medidas] --> /wheel/odometry  (+ wheel_vel/* debug)

Notas de diseño (heredadas de tus dos nodos):
  * publish_tf = False por defecto: el TF odom->base_link lo emite el EKF local.
    NO publiques el TF desde dos nodos a la vez.
  * En skid-steer las ruedas patinan: la odometria reporta vx con covarianza baja
    (confiable) y el giro (vyaw) con covarianza enorme para que el EKF lo ignore
    y el yaw lo mande el giroscopio del BNO085.
  * Failsafe doble: aqui dejamos de mandar SP (van a 0) si no llega cmd_vel, y el
    firmware ademas tiene su propio timeout (~0.46 s). Cinturon y tirantes.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray
import serial
from tf2_ros import TransformBroadcaster


class RoverBaseNode(Node):
    def __init__(self):
        super().__init__('rover_base')

        # ─────────── Parámetros de serie ───────────
        self.declare_parameter('port', '/dev/rover_enc')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('send_rate', 50.0)          # [Hz] envío de SP:
        self.declare_parameter('publish_debug', True)      # publica wheel_vel/* y wheel_debug

        # ─────────── Parámetros físicos (rover_base.yaml) ───────────
        self.declare_parameter('wheel_radius', 0.10)       # [m]
        self.declare_parameter('track_width', 0.845)       # [m] vía izq<->der
        self.declare_parameter('max_wheel_rpm', 200.0)     # clamp de seguridad
        self.declare_parameter('rpm_correction', 1.0)      # compensa PPR firmware
        self.declare_parameter('odom_rate', 50.0)          # [Hz] integración odom
        self.declare_parameter('cmd_timeout', 0.5)         # [s] sin cmd_vel -> 0
        self.declare_parameter('publish_tf', False)        # lo hace el EKF
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # Serie
        self.port      = self.get_parameter('port').value
        self.baud      = int(self.get_parameter('baud').value)
        self.send_rate = float(self.get_parameter('send_rate').value)
        self.debug     = bool(self.get_parameter('publish_debug').value)

        # Físicos
        self.R          = self.get_parameter('wheel_radius').value
        self.track      = self.get_parameter('track_width').value
        self.max_rpm    = self.get_parameter('max_wheel_rpm').value
        self.rpm_corr   = self.get_parameter('rpm_correction').value
        self.rate       = self.get_parameter('odom_rate').value
        self.cmd_to     = self.get_parameter('cmd_timeout').value
        self.pub_tf     = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # m/s por RPM = circunferencia / 60
        self.mps_per_rpm = (2.0 * math.pi * self.R) / 60.0

        # ─────────── Estado ───────────
        self.sp_l = 0.0          # setpoint a enviar (RPM), calculado por cinemática
        self.sp_r = 0.0
        self.v_left_rpm  = 0.0   # velocidad MEDIDA (encoders, RPM)
        self.v_right_rpm = 0.0
        self.x = self.y = self.th = 0.0
        self.last_cmd_time  = self.get_clock().now()
        self.last_odom_time = self.get_clock().now()

        # ─────────── I/O ROS ───────────
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry', 10)
        if self.debug:
            self.vel_l_pub = self.create_publisher(Float64, 'wheel_vel/left', 10)
            self.vel_r_pub = self.create_publisher(Float64, 'wheel_vel/right', 10)
            self.dbg_pub   = self.create_publisher(Float64MultiArray, 'wheel_debug', 10)
        self.tf_bc = TransformBroadcaster(self) if self.pub_tf else None

        # ─────────── Serie con reconexión ───────────
        self.ser = None
        self._open_serial()

        # Hilo de lectura (serie -> velocidades), timer de escritura (SP:), timer de odom
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()
        self.create_timer(1.0 / self.send_rate, self.send_setpoints)
        self.create_timer(1.0 / self.rate, self.update_odom)

        self.get_logger().info(
            f'rover_base en {self.port} @ {self.baud} | '
            f'R={self.R}m vía={self.track}m max={self.max_rpm}RPM publish_tf={self.pub_tf}')

    # ═══════════════════════ SERIE ═══════════════════════
    def _open_serial(self):
        while rclpy.ok():
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                self.get_logger().info(f'serie abierta en {self.port}')
                return
            except serial.SerialException as e:
                self.get_logger().warn(f'no abre {self.port}: {e} — reintento en 1s')
                time.sleep(1.0)

    def read_loop(self):
        """Lee CSV de 9 campos: pos_L,vel_L,pos_R,vel_R,t,sp_L,sp_R,u_L,u_R."""
        while rclpy.ok():
            try:
                raw = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not raw:
                    continue
                parts = raw.split(',')
                if len(parts) != 9:
                    continue
                vals = [float(p) for p in parts]

                self.v_left_rpm  = vals[1]
                self.v_right_rpm = vals[3]

                if self.debug:
                    ml = Float64(); ml.data = vals[1]; self.vel_l_pub.publish(ml)
                    mr = Float64(); mr.data = vals[3]; self.vel_r_pub.publish(mr)
                    d = Float64MultiArray(); d.data = vals; self.dbg_pub.publish(d)

            except (ValueError, UnicodeDecodeError):
                continue
            except serial.SerialException as e:
                self.get_logger().warn(f'serie caída: {e} — reabriendo')
                self._open_serial()
            except Exception as e:
                self.get_logger().warn(f'error lectura: {e}')

    def send_setpoints(self):
        """Manda ambos setpoints en un solo comando: SP:L,R."""
        try:
            self.ser.write(f'SP:{self.sp_l:.2f},{self.sp_r:.2f}\n'.encode('utf-8'))
        except serial.SerialException:
            pass  # la reconexión la maneja read_loop

    # ═══════════════ CINEMÁTICA DIRECTA: cmd_vel -> setpoints ═══════════════
    def cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x        # [m/s] avance
        w = msg.angular.z       # [rad/s] giro (+ = CCW / izquierda)
        v_l = v - w * self.track / 2.0
        v_r = v + w * self.track / 2.0

        self.sp_l = self._clamp(v_l / self.mps_per_rpm)
        self.sp_r = self._clamp(v_r / self.mps_per_rpm)
        self.last_cmd_time = self.get_clock().now()

    def _clamp(self, rpm):
        return max(-self.max_rpm, min(self.max_rpm, rpm))

    # ═══════════════ CINEMÁTICA INVERSA: encoders -> Odometry ═══════════════
    def update_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds * 1e-9
        self.last_odom_time = now
        if dt <= 0.0:
            return

        # Watchdog de comando: sin cmd_vel reciente -> setpoints a 0
        if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_to:
            self.sp_l = 0.0
            self.sp_r = 0.0

        # RPM medidas -> m/s
        v_l = self.v_left_rpm  * self.mps_per_rpm * self.rpm_corr
        v_r = self.v_right_rpm * self.mps_per_rpm * self.rpm_corr

        v = (v_r + v_l) / 2.0                 # avance [m/s]
        w = (v_r - v_l) / self.track          # giro [rad/s] (poco fiable: patina)

        self.x  += v * math.cos(self.th) * dt
        self.y  += v * math.sin(self.th) * dt
        self.th += w * dt

        self._publish_odom(now, v, w)

    def _publish_odom(self, stamp, v, w):
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = self._yaw_to_quat(self.th)

        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = w

        ok, huge, big = 1e-3, 1e6, 1e-3
        odom.pose.covariance[0]  = ok          # x
        odom.pose.covariance[7]  = ok          # y
        odom.pose.covariance[35] = huge        # yaw (no confiable en skid-steer)
        odom.twist.covariance[0]  = big        # vx  (CONFIABLE)
        odom.twist.covariance[35] = huge       # vyaw (lo aporta el giroscopio)

        self.odom_pub.publish(odom)

        if self.tf_bc is not None:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id  = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = self._yaw_to_quat(self.th)
            self.tf_bc.sendTransform(t)

    @staticmethod
    def _yaw_to_quat(yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = RoverBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # frena antes de salir
        try:
            node.ser.write(b'SP:0.00,0.00\n')
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()