#!/usr/bin/env python3
"""
skid_steer_base.py   ──  paquete: circ_rover_navigation

El "cerebro" cinemático del rover skid-steer (6 ruedas, 3 por lado, 1 encoder
por lado). Hace dos trabajos que comparten las MISMAS constantes:

  DIRECTA  (comando):   /cmd_vel (Twist)  ->  cmd_wheel/left,  cmd_wheel/right [RPM]
  INVERSA  (odometría): wheel_vel/left,  wheel_vel/right [RPM]  ->  /wheel/odometry

Notas de diseño para tu caso:
  • En skid-steer las ruedas PATINAN al girar, así que el rumbo por encoders no
    es confiable. Por eso la odometría reporta vx con covarianza BAJA (confiable)
    pero el giro (vyaw) con covarianza ALTA: el EKF lo ignora y deja que el
    giroscopio del BNO085 sea el dueño del yaw. La pose angular integrada aquí es
    solo para depuración/uso aislado.
  • publish_tf por defecto es FALSE: el TF odom->base_link lo emite el EKF local.
    NO lo publiques desde dos nodos a la vez.
  • Todas las constantes físicas son parámetros (rover_base.yaml).
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster


class SkidSteerBase(Node):
    def __init__(self):
        super().__init__('skid_steer_base')

        # ─────────── Parámetros físicos (cambian con el hardware) ───────────
        self.declare_parameter('wheel_radius', 0.10)        # [m]
        self.declare_parameter('track_width', 0.845)        # [m] vía izq<->der
        self.declare_parameter('max_wheel_rpm', 200.0)      # clamp de seguridad
        self.declare_parameter('rpm_correction', 1.0)       # ver nota PPR abajo
        self.declare_parameter('odom_rate', 50.0)           # [Hz] integración
        self.declare_parameter('cmd_timeout', 0.5)          # [s] sin cmd_vel -> 0
        self.declare_parameter('publish_tf', False)         # lo hace el EKF
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

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
        self.x = self.y = self.th = 0.0
        self.v_left_rpm = 0.0     # velocidad MEDIDA (de los encoders)
        self.v_right_rpm = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_odom_time = self.get_clock().now()

        # ─────────── I/O ───────────
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Float64, 'wheel_vel/left',
                                 lambda m: setattr(self, 'v_left_rpm', m.data), 10)
        self.create_subscription(Float64, 'wheel_vel/right',
                                 lambda m: setattr(self, 'v_right_rpm', m.data), 10)

        self.cmd_left_pub  = self.create_publisher(Float64, 'cmd_wheel/left', 10)
        self.cmd_right_pub = self.create_publisher(Float64, 'cmd_wheel/right', 10)
        self.odom_pub      = self.create_publisher(Odometry, '/wheel/odometry', 10)

        self.tf_bc = TransformBroadcaster(self) if self.pub_tf else None

        self.create_timer(1.0 / self.rate, self.update)

        self.get_logger().info(
            f'skid_steer_base: R={self.R}m, vía={self.track}m, '
            f'max={self.max_rpm}RPM, publish_tf={self.pub_tf}')

    # ───────────────── CINEMÁTICA DIRECTA: cmd_vel -> setpoints ─────────────────
    def cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x        # [m/s] avance
        w = msg.angular.z       # [rad/s] giro (+ = CCW / izquierda)

        # velocidad de cada lado en m/s
        v_l = v - w * self.track / 2.0
        v_r = v + w * self.track / 2.0

        # m/s -> RPM, con clamp
        rpm_l = self._clamp(v_l / self.mps_per_rpm)
        rpm_r = self._clamp(v_r / self.mps_per_rpm)

        self._send(self.cmd_left_pub,  rpm_l)
        self._send(self.cmd_right_pub, rpm_r)
        self.last_cmd_time = self.get_clock().now()

    def _clamp(self, rpm):
        return max(-self.max_rpm, min(self.max_rpm, rpm))

    def _send(self, pub, rpm):
        m = Float64(); m.data = float(rpm); pub.publish(m)

    # ───────────────── CINEMÁTICA INVERSA: encoders -> Odometry ─────────────────
    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds * 1e-9
        self.last_odom_time = now
        if dt <= 0.0:
            return

        # Watchdog de comando: sin cmd_vel reciente -> frena
        if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_to:
            self._send(self.cmd_left_pub, 0.0)
            self._send(self.cmd_right_pub, 0.0)

        # RPM medidas -> m/s (rpm_correction compensa un PPR de firmware mal puesto)
        v_l = self.v_left_rpm  * self.mps_per_rpm * self.rpm_corr
        v_r = self.v_right_rpm * self.mps_per_rpm * self.rpm_corr

        v = (v_r + v_l) / 2.0                 # avance [m/s]
        w = (v_r - v_l) / self.track          # giro [rad/s]  (poco fiable: patina)

        # Integración de la pose (solo para debug / uso aislado)
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

        # Covarianzas: confiamos en vx, NO en el giro de ruedas (lo da el IMU).
        # Diagonal 6x6: [x, y, z, roll, pitch, yaw]
        big, ok, huge = 1e-3, 1e-3, 1e6
        odom.pose.covariance[0]  = ok          # x
        odom.pose.covariance[7]  = ok          # y
        odom.pose.covariance[35] = huge        # yaw (no confiable en skid-steer)
        odom.twist.covariance[0]  = big        # vx  (CONFIABLE)
        odom.twist.covariance[35] = huge       # vyaw (que lo aporte el giroscopio)

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
    node = SkidSteerBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()