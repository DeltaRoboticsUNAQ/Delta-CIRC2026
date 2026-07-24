#!/usr/bin/env python3
"""
rover_comms_bridge.py  ──  circ_rover_hardware/circ_rover_hardware/

Nodo unificado para la arquitectura de 3 STM32 + CAN (100% CAN entre STM).

    IMU STM32 ──┐
                ├── CAN 500k ──> COMMS STM32 ── USART2 460800 ──> Jetson
    ENC STM32 ──┘                    ▲                              │
                                     └──────── SP: (comandos) ──────┘

Un solo puerto serie (la STM de comunicaciones). El nodo:
  - LEE telemetria (IMU 10 campos + encoders 9 campos) y publica los topics.
  - ESCRIBE "SP:L,R" que la comms reenvia por CAN 0x300 al chasis.

*** VERSION CON PRINT DE DIAGNOSTICO EN send_cmd ***
Busca la linea ">>> enviando: ..." en la consola para ver que se escribe.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Bool, Float32MultiArray

import serial


CYAN, GREEN, YELLOW, RED = '\033[96m', '\033[92m', '\033[93m', '\033[91m'
BOLD, DIM, RESET = '\033[1m', '\033[2m', '\033[0m'


def quat_to_euler_deg(qw, qx, qy, qz):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    d = math.degrees
    return d(roll), d(pitch), d(yaw)


def yaw_to_quat(yaw):
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q


class RoverCommsBridge(Node):

    def __init__(self):
        super().__init__('rover_comms_bridge')

        # ── Puertos ───────────────────────────────────────────────────
        self.declare_parameter('telemetry_port', '/dev/ttyACM0')
        self.declare_parameter('telemetry_baud', 460800)
        self.declare_parameter('cmd_port', '/dev/ttyACM0')
        self.declare_parameter('cmd_baud', 460800)

        # ── Cinematica ────────────────────────────────────────────────
        self.declare_parameter('wheel_radius', 0.10)
        self.declare_parameter('track_width', 0.845)

        # ── Frames ────────────────────────────────────────────────────
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', False)

        # ── Seguridad / watchdog ──────────────────────────────────────
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('cmd_rate_hz', 20.0)
        self.declare_parameter('data_timeout_s', 1.0)
        self.declare_parameter('reconnect_period_s', 2.0)

        self.R = float(self.get_parameter('wheel_radius').value)
        self.track = float(self.get_parameter('track_width').value)
        self.imu_frame = self.get_parameter('imu_frame_id').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value

        # ── Publicadores ──────────────────────────────────────────────
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry', 10)
        self.debug_pub = self.create_publisher(Float32MultiArray, '/wheel_debug', 10)
        self.imu_health_pub = self.create_publisher(Bool, '/imu/bridge_healthy', 10)
        self.enc_health_pub = self.create_publisher(Bool, '/wheel/bridge_healthy', 10)

        # ── Suscriptor de comandos ────────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # ── Estado ────────────────────────────────────────────────────
        self.tel_ser = None
        self.cmd_ser = None
        self.same_port = (self.get_parameter('cmd_port').value ==
                          self.get_parameter('telemetry_port').value)

        self.last_imu_time = None
        self.last_enc_time = None
        self._boot = time.monotonic()
        self._last_reconnect = 0.0
        self._imu_dead_logged = False
        self._enc_dead_logged = False

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_stamp = None

        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_time = 0.0

        self._imu_count = 0
        self._enc_count = 0
        self._dbg_last = 0.0   # para no spamear el print de send_cmd

        print(f'{BOLD}{CYAN}Rover Comms Bridge{RESET}  '
              f'{DIM}tel: {self.get_parameter("telemetry_port").value} @ '
              f'{self.get_parameter("telemetry_baud").value}  |  '
              f'cmd: {self.get_parameter("cmd_port").value} @ '
              f'{self.get_parameter("cmd_baud").value}  '
              f'same_port={self.same_port}{RESET}\n')

        self.try_connect()

        # ── Timers ────────────────────────────────────────────────────
        self.create_timer(0.02, self.read_serial)
        self.create_timer(1.0 / float(self.get_parameter('cmd_rate_hz').value),
                          self.send_cmd)
        self.create_timer(0.25, self.watchdog)

    # ══════════════════════════════════════════════════════════════════
    def try_connect(self):
        tel_port = self.get_parameter('telemetry_port').value
        tel_baud = int(self.get_parameter('telemetry_baud').value)
        cmd_port = self.get_parameter('cmd_port').value
        cmd_baud = int(self.get_parameter('cmd_baud').value)

        if self.tel_ser is None:
            try:
                self.tel_ser = serial.Serial(tel_port, tel_baud, timeout=0.05)
                self.tel_ser.reset_input_buffer()
                self.last_imu_time = None
                self.last_enc_time = None
                self._boot = time.monotonic()
                self.get_logger().info(f'Telemetria abierta: {tel_port} @ {tel_baud}')
            except (serial.SerialException, OSError) as e:
                self.tel_ser = None
                self.get_logger().warn(f'No se pudo abrir telemetria {tel_port}: {e}',
                                       throttle_duration_sec=10.0)

        # Mismo puerto -> reutilizar el objeto serial (no abrir dos veces).
        if self.same_port:
            self.cmd_ser = self.tel_ser
            return

        if self.cmd_ser is None:
            try:
                self.cmd_ser = serial.Serial(cmd_port, cmd_baud, timeout=0.05)
                self.get_logger().info(f'Comandos abierto: {cmd_port} @ {cmd_baud}')
            except (serial.SerialException, OSError) as e:
                self.cmd_ser = None
                self.get_logger().warn(f'No se pudo abrir comandos {cmd_port}: {e}',
                                       throttle_duration_sec=10.0)

    def close_ports(self):
        for s in {self.tel_ser, self.cmd_ser}:
            if s is not None:
                try:
                    s.close()
                except Exception:
                    pass
        self.tel_ser = None
        self.cmd_ser = None

    # ══════════════════════════════════════════════════════════════════
    def read_serial(self):
        if self.tel_ser is None or not self.tel_ser.is_open:
            return
        try:
            while self.tel_ser.in_waiting > 0:
                linea = self.tel_ser.readline().decode('utf-8', errors='ignore').strip()
                if not linea:
                    continue
                campos = linea.split(',')
                n = len(campos)
                if n not in (9, 10):
                    continue
                try:
                    datos = [float(v) for v in campos]
                except ValueError:
                    continue

                if n == 10:
                    self.last_imu_time = time.monotonic()
                    self.handle_imu(datos)
                else:
                    self.last_enc_time = time.monotonic()
                    self.handle_encoders(datos)

        except (serial.SerialException, OSError) as e:
            print()
            self.get_logger().error(f'Puerto de telemetria perdido: {e}')
            if self.same_port:
                self.cmd_ser = None
            try:
                self.tel_ser.close()
            except Exception:
                pass
            self.tel_ser = None

    # ── IMU: 10 campos ────────────────────────────────────────────────
    def handle_imu(self, d):
        qw, qx, qy, qz, gx, gy, gz, ax, ay, az = d
        self._imu_count += 1

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        msg.orientation.w = qw
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation_covariance = [0.005, 0.0, 0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.05]
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance = [0.002, 0.0, 0.0, 0.0, 0.002, 0.0, 0.0, 0.0, 0.002]
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance = [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]
        self.imu_pub.publish(msg)
        self.print_pretty(qw, qx, qy, qz, gz)

    # ── Encoders: 9 campos ────────────────────────────────────────────
    def handle_encoders(self, d):
        pos_L, vel_L, pos_R, vel_R, t_stm, sp_L, sp_R, u_L, u_R = d
        self._enc_count += 1

        dbg = Float32MultiArray()
        dbg.data = [float(v) for v in d]
        self.debug_pub.publish(dbg)

        v_l = vel_L * 2.0 * math.pi * self.R / 60.0
        v_r = vel_R * 2.0 * math.pi * self.R / 60.0
        v = (v_l + v_r) / 2.0
        w = (v_r - v_l) / self.track

        now = self.get_clock().now()
        if self.last_odom_stamp is None:
            self.last_odom_stamp = now
            return
        dt = (now - self.last_odom_stamp).nanoseconds * 1e-9
        self.last_odom_stamp = now
        if dt <= 0.0 or dt > 0.5:
            return

        self.theta += w * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quat(self.theta)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        pc = [0.0] * 36
        pc[0] = 0.05; pc[7] = 0.05; pc[14] = 1e6
        pc[21] = 1e6; pc[28] = 1e6; pc[35] = 1.0
        odom.pose.covariance = pc
        tc = [0.0] * 36
        tc[0] = 0.02; tc[7] = 1e6; tc[14] = 1e6
        tc[21] = 1e6; tc[28] = 1e6; tc[35] = 0.5
        odom.twist.covariance = tc

        self.odom_pub.publish(odom)

    # ══════════════════════════════════════════════════════════════════
    def cmd_vel_cb(self, msg):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z
        self.last_cmd_time = time.monotonic()

    def send_cmd(self):
        if self.cmd_ser is None or not self.cmd_ser.is_open:
            # DIAGNOSTICO: si esto se ve, el puerto de comandos no esta abierto.
            now = time.monotonic()
            if now - self._dbg_last > 1.0:
                self._dbg_last = now
                print(f'\n{RED}>>> send_cmd: cmd_ser NO disponible '
                      f'(None={self.cmd_ser is None}){RESET}')
            return

        timeout = float(self.get_parameter('cmd_timeout_s').value)
        stale = (time.monotonic() - self.last_cmd_time) > timeout
        if stale:
            v, w = 0.0, 0.0
        else:
            v, w = self.cmd_v, self.cmd_w

        v_l = v - w * self.track / 2.0
        v_r = v + w * self.track / 2.0
        rpm_l = v_l * 60.0 / (2.0 * math.pi * self.R)
        rpm_r = v_r * 60.0 / (2.0 * math.pi * self.R)

        try:
            linea = f'SP:{rpm_l:.2f},{rpm_r:.2f}\n'
            self.cmd_ser.write(linea.encode('ascii'))

            # DIAGNOSTICO: imprime como maximo 1 vez por segundo que se escribe.
            now = time.monotonic()
            if now - self._dbg_last > 1.0:
                self._dbg_last = now
                estado = 'STALE(watchdog->0)' if stale else 'OK'
                print(f'\n{GREEN}>>> enviando: {linea.strip()}  '
                      f'[{estado}]  cmd_v={self.cmd_v:.3f} cmd_w={self.cmd_w:.3f}{RESET}')
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Puerto de comandos perdido: {e}')
            if not self.same_port:
                self.cmd_ser = None

    # ══════════════════════════════════════════════════════════════════
    def watchdog(self):
        now = time.monotonic()
        timeout = float(self.get_parameter('data_timeout_s').value)
        reconnect = float(self.get_parameter('reconnect_period_s').value)
        connected = self.tel_ser is not None and self.tel_ser.is_open

        imu_ok = connected and self.last_imu_time is not None and (now - self.last_imu_time) <= timeout
        enc_ok = connected and self.last_enc_time is not None and (now - self.last_enc_time) <= timeout

        m = Bool(); m.data = imu_ok; self.imu_health_pub.publish(m)
        m = Bool(); m.data = enc_ok; self.enc_health_pub.publish(m)

        if not connected and (now - self._last_reconnect) >= reconnect:
            self._last_reconnect = now
            self.try_connect()

    # ══════════════════════════════════════════════════════════════════
    def print_pretty(self, qw, qx, qy, qz, gz):
        roll, pitch, yaw = quat_to_euler_deg(qw, qx, qy, qz)
        line = (f'{GREEN}R{RESET}{roll:+6.1f}° '
                f'{GREEN}P{RESET}{pitch:+6.1f}° '
                f'{GREEN}Y{RESET}{yaw:+6.1f}°  '
                f'{CYAN}ωz{RESET}{math.degrees(gz):+6.1f}°/s  '
                f'{YELLOW}odom{RESET} x{self.x:+5.2f} y{self.y:+5.2f}  '
                f'{DIM}imu#{self._imu_count} enc#{self._enc_count}{RESET}')
        print(f'\r{line}', end='', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = RoverCommsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.close_ports()
        node.get_logger().info('Puertos cerrados.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()