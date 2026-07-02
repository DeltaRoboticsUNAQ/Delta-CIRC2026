#!/usr/bin/env python3
"""
heading_calibration.py   ──  circ_rover_navigation

Calibra el heading absoluto SIN magnetómetro, usando el movimiento del GPS.

Problema: el BNO085 en Game RV da un yaw RELATIVO (arranca en 0 donde booteó),
así que el navsat_transform no sabe dónde está el norte. Este nodo lo resuelve:

  1. Avanza el rover en línea recta unos metros.
  2. Con dos posiciones GPS calcula el rumbo REAL (course-over-ground, ya
     referenciado al norte verdadero).
  3. Lee el yaw que cree tener el giroscopio en ese momento.
  4. La diferencia es el `yaw_offset` que le falta al navsat_transform.

SEGURIDAD: no maneja solo al arrancar. Espera a que lo dispares:
    ros2 service call /calibrate_heading std_srvs/srv/Trigger

Necesita el rover capaz de moverse (encoders reales o fake_motors) + GPS con fix.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_srvs.srv import Trigger


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class HeadingCalibration(Node):
    def __init__(self):
        super().__init__('heading_calibration')

        # ── Parámetros ──
        self.declare_parameter('drive_speed', 0.3)     # [m/s] velocidad de avance
        self.declare_parameter('drive_distance', 3.0)  # [m] distancia a recorrer
        self.declare_parameter('min_distance', 1.5)    # [m] mínimo válido (ruido GPS)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('fix_topic', '/fix')
        self.declare_parameter('imu_topic', '/imu/data_raw')

        self.speed    = self.get_parameter('drive_speed').value
        self.distance = self.get_parameter('drive_distance').value
        self.min_dist = self.get_parameter('min_distance').value

        # ── I/O ──
        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.create_subscription(
            NavSatFix, self.get_parameter('fix_topic').value, self.fix_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self.imu_cb,
            qos_profile_sensor_data)
        self.srv = self.create_service(Trigger, 'calibrate_heading', self.on_trigger)

        # ── Estado ──
        self.last_fix = None        # (lat, lon)
        self.last_yaw = None        # yaw del IMU
        self.start_fix = None
        self.start_yaw = None
        self.calibrating = False

        self.get_logger().info(
            'heading_calibration listo. Dispara con:\n'
            '   ros2 service call /calibrate_heading std_srvs/srv/Trigger')

    def fix_cb(self, msg: NavSatFix):
        if msg.status.status >= 0:           # solo fixes válidos
            self.last_fix = (msg.latitude, msg.longitude)

    def imu_cb(self, msg: Imu):
        self.last_yaw = quat_to_yaw(msg.orientation)

    def on_trigger(self, request, response):
        if self.calibrating:
            response.success = False
            response.message = 'Ya hay una calibración en curso.'
            return response
        if self.last_fix is None:
            response.success = False
            response.message = 'Sin fix GPS válido. Espera a que el GPS enganche.'
            return response
        if self.last_yaw is None:
            response.success = False
            response.message = 'Sin datos de IMU. ¿Está corriendo el bridge?'
            return response

        # Guarda el punto de partida y arranca el avance
        self.start_fix = self.last_fix
        self.start_yaw = self.last_yaw
        self.calibrating = True
        self.get_logger().info('Calibrando: avanzando en recta...')
        self.timer = self.create_timer(0.05, self.drive_step)

        response.success = True
        response.message = ('Calibración iniciada. El resultado saldrá en el log '
                            'cuando termine el avance.')
        return response

    def drive_step(self):
        dist = self._distance(self.start_fix, self.last_fix)

        if dist < self.distance:
            # sigue avanzando
            tw = Twist(); tw.linear.x = self.speed
            self.cmd_pub.publish(tw)
            return

        # Llegó a la distancia: frena y calcula
        self.cmd_pub.publish(Twist())        # parar
        self.timer.cancel()
        self.calibrating = False
        self._compute(dist)

    def _compute(self, dist):
        if dist < self.min_dist:
            self.get_logger().error(
                f'Solo se avanzó {dist:.2f} m (< {self.min_dist} m). El ruido del '
                f'GPS domina; calibración NO confiable. Aumenta drive_distance o '
                f'revisa que el rover se mueva.')
            return

        # Rumbo real (ENU: 0 = Este, CCW positivo) entre inicio y fin
        true_heading = self._enu_heading(self.start_fix, self.last_fix)
        # yaw que tenía el giroscopio al inicio del avance
        imu_yaw = self.start_yaw
        # El offset que le falta al navsat_transform
        yaw_offset = self._normalize(true_heading - imu_yaw)

        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════════════════════════╗\n'
            '║              CALIBRACIÓN DE HEADING COMPLETA             ║\n'
            '╠══════════════════════════════════════════════════════════╣\n'
            f'║  Distancia recorrida : {dist:6.2f} m                       \n'
            f'║  Rumbo real (GPS)    : {math.degrees(true_heading):7.2f}°  \n'
            f'║  Yaw giroscopio      : {math.degrees(imu_yaw):7.2f}°       \n'
            f'║  >>> yaw_offset      : {yaw_offset:7.4f} rad ({math.degrees(yaw_offset):.2f}°)\n'
            '╠══════════════════════════════════════════════════════════╣\n'
            '║  Pon este valor en navsat_transform.yaml:                ║\n'
            f'║      yaw_offset: {yaw_offset:.4f}                          \n'
            '║  y relanza el navsat_transform (sin reiniciar la IMU).   ║\n'
            '╚══════════════════════════════════════════════════════════╝')

    # ── Utilidades geográficas ──
    @staticmethod
    def _distance(a, b):
        """Distancia plana aproximada entre dos (lat,lon) en metros."""
        if a is None or b is None:
            return 0.0
        R = 6378137.0
        dlat = math.radians(b[0] - a[0])
        dlon = math.radians(b[1] - a[1])
        dn = dlat * R
        de = dlon * R * math.cos(math.radians(a[0]))
        return math.hypot(dn, de)

    @staticmethod
    def _enu_heading(a, b):
        """Rumbo ENU (0=Este, CCW+) del vector a->b."""
        R = 6378137.0
        dn = math.radians(b[0] - a[0]) * R
        de = math.radians(b[1] - a[1]) * R * math.cos(math.radians(a[0]))
        return math.atan2(dn, de)

    @staticmethod
    def _normalize(ang):
        return math.atan2(math.sin(ang), math.cos(ang))


def main(args=None):
    rclpy.init(args=args)
    node = HeadingCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()