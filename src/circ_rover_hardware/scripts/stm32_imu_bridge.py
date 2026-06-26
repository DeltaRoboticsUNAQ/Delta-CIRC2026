#!/usr/bin/env python3
"""
Bridge serie -> ROS2 para el BNO085 sobre STM32 (v2: mensaje Imu completo).

Lee del puerto serie lineas CSV de 10 campos con el formato:
    qw,qx,qy,qz,gx,gy,gz,ax,ay,az
y publica un sensor_msgs/Imu COMPLETO en /imu/data_raw:

  - qw,qx,qy,qz -> msg.orientation           (Game Rotation Vector)
  - gx,gy,gz    -> msg.angular_velocity      (rad/s, giroscopio calibrado)
  - ax,ay,az    -> msg.linear_acceleration   (m/s^2, acelerometro)

Consumidores:
  - UnifiedSafetyController: orientation (pitch/roll) + linear_acceleration.z
  - robot_localization (EKF local): angular_velocity + orientation

NOTA DE BAUDRATE: el firmware manda ~80 bytes/linea a 100 Hz (~8 kB/s).
USART2 del STM32 y este bridge deben ir ambos a 460800 baud; a 115200
el bus se satura.

── Watchdog + reconexion ────────────────────────────────────────────────
  - Watchdog: si no llegan lineas validas en `data_timeout_s`, el bridge
    declara la IMU muerta: lo loguea UNA vez, muestra una linea roja en
    vivo y publica False en `health_topic`.
  - Reconexion automatica: si el puerto nunca abrio o se cae a media
    operacion (cable desconectado), reintenta cada `reconnect_period_s`
    sin tumbar el nodo.
  - Puerto colgado: si el puerto sigue abierto pero no llegan datos por
    mas de 5 * data_timeout_s (firmware congelado), se cierra y se
    vuelve a abrir solo.
  - `health_topic` (std_msgs/Bool, ~4 Hz): True = datos frescos fluyendo.
    El controlador NO debe depender solo de este topico (si el bridge
    muere, el topico tambien); la proteccion real es el timeout por
    timestamp en el propio controlador.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
import serial


# ── Codigos ANSI para la salida bonita ────────────────────────────────
CYAN   = '\033[96m'
GREEN  = '\033[92m'
YELLOW = '\033[93m'
RED    = '\033[91m'
BOLD   = '\033[1m'
DIM    = '\033[2m'
RESET  = '\033[0m'


def quat_to_euler_deg(qw, qx, qy, qz):
    """Cuaternion -> (roll, pitch, yaw) en grados."""
    # Roll (x)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))  # clamp por seguridad numerica
    pitch = math.asin(sinp)

    # Yaw (z)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    r = math.degrees
    return r(roll), r(pitch), r(yaw)


class STM32ImuBridge(Node):
    def __init__(self):
        super().__init__('stm32_imu_bridge')

        # ── Parametros ────────────────────────────────────────────────
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 460800)         # ¡igual que USART2!
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('topic', '/imu/data_raw')
        self.declare_parameter('health_topic', '/imu/bridge_healthy')
        self.declare_parameter('data_timeout_s', 1.0)      # sin datos validos -> IMU muerta
        self.declare_parameter('reconnect_period_s', 2.0)  # reintento de apertura del puerto

        self.frame_id = self.get_parameter('frame_id').value
        topic         = self.get_parameter('topic').value
        health_topic  = self.get_parameter('health_topic').value
        port          = self.get_parameter('port').value
        baudrate      = self.get_parameter('baudrate').value

        # ── Publicadores ──────────────────────────────────────────────
        # IMU: perfil de sensor (best-effort, depth corto) -> alineado con el
        # SensorDataQoS() del UnifiedSafetyController. Para datos a 100 Hz
        # interesa el dato fresco, no reintentar entregas viejas.
        self.publisher_  = self.create_publisher(Imu, topic, qos_profile_sensor_data)
        # Salud: reliable normal, no es un stream de sensor de alta frecuencia.
        self.health_pub_ = self.create_publisher(Bool, health_topic, 10)

        # ── Estado del watchdog ───────────────────────────────────────
        self.serial_port = None
        self.last_data_time = None          # monotonic de la ultima linea valida
        self.healthy = False
        self._dead_logged = False
        self._boot_time = time.monotonic()  # para el periodo de gracia al arrancar
        self._last_reconnect_attempt = 0.0
        self._msg_count = 0

        # Cabecera de la salida bonita (una sola vez)
        print(f'{BOLD}{CYAN}STM32 IMU Bridge v2{RESET}'
              f'  {DIM}{port} @ {baudrate} baud  ->  {topic}{RESET}\n')

        # Primer intento de conexion (si falla, el watchdog reintenta)
        self.try_connect()

        # ── Timers ────────────────────────────────────────────────────
        self.read_timer     = self.create_timer(0.02, self.read_serial_data)  # ~50 Hz
        self.watchdog_timer = self.create_timer(0.25, self.watchdog)          # ~4 Hz

    # ══════════════════════════════════════════════════════════════════
    # Manejo del puerto serie
    # ══════════════════════════════════════════════════════════════════
    def try_connect(self):
        """Intenta abrir el puerto. Devuelve True si quedo abierto."""
        port     = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=0.05)
            self.serial_port.reset_input_buffer()  # descartar basura acumulada
            # Nuevo periodo de gracia: recien abierto, dar tiempo a que fluyan datos
            self.last_data_time = None
            self._boot_time = time.monotonic()
            self.get_logger().info(f'Puerto serie abierto: {port} @ {baudrate}')
            return True
        except (serial.SerialException, OSError) as e:
            self.serial_port = None
            self.get_logger().warn(f'No se pudo abrir {port}: {e}',
                                   throttle_duration_sec=10.0)
            return False

    def close_port(self):
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None

    # ══════════════════════════════════════════════════════════════════
    # Lectura serie (~50 Hz, drena todo el buffer)
    # ══════════════════════════════════════════════════════════════════
    def read_serial_data(self):
        if self.serial_port is None or not self.serial_port.is_open:
            return

        try:
            while self.serial_port.in_waiting > 0:
                linea = self.serial_port.readline().decode(
                    'utf-8', errors='ignore').strip()

                if not linea:
                    continue

                valores = linea.split(',')

                # Solo lineas con 10 campos son datos. Los mensajes de debug
                # del firmware ("INT=...", "sh2_open rc=...") se descartan.
                if len(valores) != 10:
                    continue

                try:
                    datos = [float(v) for v in valores]
                except ValueError:
                    continue

                qw, qx, qy, qz, gx, gy, gz, ax, ay, az = datos

                self.last_data_time = time.monotonic()  # alimenta al watchdog
                self.publish_imu(qw, qx, qy, qz, gx, gy, gz, ax, ay, az)
                self.print_pretty(qw, qx, qy, qz, gz, az)

        except (serial.SerialException, OSError) as e:
            # Cable desconectado a media operacion: cerrar y dejar que el
            # watchdog se encargue de reconectar.
            print()  # romper la linea viva
            self.get_logger().error(f'Puerto serie perdido: {e}')
            self.close_port()

    # ══════════════════════════════════════════════════════════════════
    # Watchdog (~4 Hz): salud, reconexion y reciclado de puerto colgado
    # ══════════════════════════════════════════════════════════════════
    def watchdog(self):
        now       = time.monotonic()
        timeout   = float(self.get_parameter('data_timeout_s').value)
        reconnect = float(self.get_parameter('reconnect_period_s').value)

        connected = self.serial_port is not None and self.serial_port.is_open

        # Frescura de datos. Si nunca han llegado, se mide desde el arranque
        # (o desde la ultima reapertura del puerto): periodo de gracia.
        last    = self.last_data_time if self.last_data_time is not None else self._boot_time
        elapsed = now - last
        has_data    = self.last_data_time is not None
        healthy_now = connected and has_data and elapsed <= timeout
        in_grace    = (not has_data) and elapsed <= timeout

        if healthy_now:
            if not self.healthy:
                self.get_logger().info('IMU OK: datos fluyendo.')
            self.healthy = True
            self._dead_logged = False

        elif not in_grace:
            if not self._dead_logged:
                print()  # romper la linea viva antes del log
                razon = ('puerto serie desconectado' if not connected
                         else 'puerto abierto pero sin datos (firmware colgado?)')
                self.get_logger().error(f'IMU MUERTA: {razon}')
                self._dead_logged = True
            self.healthy = False

            # Linea viva en rojo mientras este caida
            estado = 'reintentando puerto...' if not connected else 'esperando datos...'
            print(f'\r{RED}{BOLD}IMU SIN DATOS{RESET}  {DIM}({estado}){RESET}'
                  f'{" " * 40}', end='', flush=True)

        # Telemetria de salud (el controlador puede suscribirse, pero su
        # proteccion real debe ser el timeout por timestamp del mensaje Imu)
        msg = Bool()
        msg.data = self.healthy
        self.health_pub_.publish(msg)

        # Puerto abierto pero congelado mucho tiempo: reciclarlo
        if connected and has_data and (now - self.last_data_time) > 5.0 * timeout:
            self.get_logger().warn('Reciclando puerto serie por inactividad prolongada...')
            self.close_port()
            connected = False

        # Reintento de conexion
        if not connected and (now - self._last_reconnect_attempt) >= reconnect:
            self._last_reconnect_attempt = now
            self.try_connect()

    # ══════════════════════════════════════════════════════════════════
    # Salida bonita y publicacion
    # ══════════════════════════════════════════════════════════════════
    def print_pretty(self, qw, qx, qy, qz, gz, az):
        """Una linea que se refresca en vivo con los datos legibles."""
        self._msg_count += 1
        roll, pitch, yaw = quat_to_euler_deg(qw, qx, qy, qz)

        line = (
            f'{GREEN}Roll{RESET} {roll:+7.1f}°  '
            f'{GREEN}Pitch{RESET} {pitch:+7.1f}°  '
            f'{GREEN}Yaw{RESET} {yaw:+7.1f}°   '
            f'{CYAN}ωz{RESET} {math.degrees(gz):+7.1f}°/s   '
            f'{YELLOW}az{RESET} {az:+6.2f} m/s²   '
            f'{DIM}#{self._msg_count}{RESET}'
        )
        # \r vuelve al inicio de la linea; end='' evita el salto.
        print(f'\r{line}', end='', flush=True)

    def publish_imu(self, qw, qx, qy, qz, gx, gy, gz, ax, ay, az):
        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Orientacion (Game Rotation Vector: roll/pitch absolutos,
        # yaw RELATIVO al arranque -- por eso su varianza es mayor).
        msg.orientation.w = qw
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation_covariance = [
            0.005, 0.0,   0.0,
            0.0,   0.005, 0.0,
            0.0,   0.0,   0.05,
        ]

        # Velocidad angular real del giroscopio calibrado (rad/s).
        # Valores de covarianza ajustables al tunear el EKF.
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance = [
            0.002, 0.0,   0.0,
            0.0,   0.002, 0.0,
            0.0,   0.0,   0.002,
        ]

        # Aceleracion lineal completa (m/s^2). az la consume el factor
        # de vibracion del safety controller; sigue siendo la cruda.
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance = [
            0.04, 0.0,  0.0,
            0.0,  0.04, 0.0,
            0.0,  0.0,  0.04,
        ]

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = STM32ImuBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()  # salto final para no dejar el cursor pegado a la linea viva
        node.close_port()
        node.get_logger().info('Puerto serie cerrado.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()