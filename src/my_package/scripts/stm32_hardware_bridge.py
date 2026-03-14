#!/usr/bin/env python3
"""
STM32 Hardware Bridge — Puente bidireccional ROS 2 ↔ STM32 para brazo robótico.

Arquitectura de nodos:
  /joint_states_cmd   →  stm32_hardware_bridge  →  STM32 (UART serial)
  STM32 (UART serial) →  stm32_hardware_bridge  →  /hardware/joint_states
  /hardware/joint_states  (opcional)             →  /joint_states (sync a sim)

Protocolo serie ROS 2 → STM32 (comando):
  CMD:B:<deg>;W1:<deg>;W2:<deg>;A1:<pct>;A2:<pct>\\r\\n
  Donde:
    B   = Base, en grados (motor DC + encoder)
    W1  = Motor diferencial muñeca 1, en grados (encoder)
    W2  = Motor diferencial muñeca 2, en grados (encoder)
    A1  = Actuador lineal 1, porcentaje extensión 0-100 (potenciómetro)
    A2  = Actuador lineal 2, porcentaje extensión 0-100 (potenciómetro)

Protocolo serie STM32 → ROS 2 (feedback):
  FB:BE:<ticks>;W1:<ticks>;W2:<ticks>;P1:<adc>;P2:<adc>\\n
  Donde:
    BE  = Encoder base (ticks acumulados con signo)
    W1  = Encoder motor muñeca 1 (ticks acumulados con signo)
    W2  = Encoder motor muñeca 2 (ticks acumulados con signo)
    P1  = ADC potenciómetro actuador 1 (0-4095 para 12 bits)
    P2  = ADC potenciómetro actuador 2 (0-4095 para 12 bits)

Cinemática diferencial de la muñeca:
  pitch = (W1 + W2) / 2     (ubracket_joint)
  roll  = (W1 - W2) / 2     (endeffector_joint)
  → Comando inverso: W1 = pitch + roll,  W2 = pitch - roll

Hardware mapeado a joints URDF:
  bracket_joint      ← base (motor DC + encoder)
  humerus_low_joint  ← actuador lineal 1 (potenciómetro)
  forearm_low_joint  ← actuador lineal 2 (potenciómetro)
  ubracket_joint     ← muñeca pitch  (diferencial, encoders W1+W2)
  endeffector_joint  ← muñeca roll   (diferencial, encoders W1-W2)
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


# ── Constantes de calibración por defecto ──────────────────────────────────────
# AJUSTAR según el hardware real antes de usar con el brazo físico.

# Encoder base y muñeca: ticks por vuelta completa (360°).
# Para un encoder de cuadratura de 500 PPR → 2000 ticks/rev.
BASE_ENCODER_TICKS_PER_REV = 2000
WRIST_ENCODER_TICKS_PER_REV = 2000

# Potenciómetro: valores ADC (12 bits STM32) en los extremos mecánicos.
# Medir con el actuador completamente retraído y extendido.
LINEAR_ACT_POT_MIN = 200    # ADC cuando actuador está en posición mínima
LINEAR_ACT_POT_MAX = 3800   # ADC cuando actuador está en posición máxima

# Rango angular del actuador en radianes (del URDF: -1.5 a 1.5 → 3.0 rad total).
LINEAR_ACT_RANGE_RAD = 3.0


class Stm32HardwareBridge(Node):
    """
    Nodo ROS 2 que actúa como puente bidireccional entre la simulación y el STM32.

    Responsabilidades:
    - Recibir comandos de posición desde /joint_states_cmd y enviarlos al STM32.
    - Leer feedback serie del STM32 (encoders + potenciómetros).
    - Convertir feedback a posiciones de joints en radianes.
    - Publicar /hardware/joint_states con las posiciones reales del hardware.
    - Opcionalmente sincronizar /hardware/joint_states → /joint_states para
      que la simulación refleje el estado real del brazo.
    """

    def __init__(self):
        super().__init__('stm32_hardware_bridge')

        # ── Parámetros ────────────────────────────────────────────────────────
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('sync_to_sim', True)
        self.declare_parameter('feedback_rate_hz', 50.0)
        self.declare_parameter('base_ticks_per_rev', BASE_ENCODER_TICKS_PER_REV)
        self.declare_parameter('wrist_ticks_per_rev', WRIST_ENCODER_TICKS_PER_REV)
        self.declare_parameter('pot_min', LINEAR_ACT_POT_MIN)
        self.declare_parameter('pot_max', LINEAR_ACT_POT_MAX)
        self.declare_parameter('actuator_range_rad', LINEAR_ACT_RANGE_RAD)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.sync_to_sim = (
            self.get_parameter('sync_to_sim').get_parameter_value().bool_value
        )
        feedback_rate = (
            self.get_parameter('feedback_rate_hz').get_parameter_value().double_value
        )
        self.base_ticks_per_rev = (
            self.get_parameter('base_ticks_per_rev').get_parameter_value().integer_value
        )
        self.wrist_ticks_per_rev = (
            self.get_parameter('wrist_ticks_per_rev').get_parameter_value().integer_value
        )
        self.pot_min = (
            self.get_parameter('pot_min').get_parameter_value().integer_value
        )
        self.pot_max = (
            self.get_parameter('pot_max').get_parameter_value().integer_value
        )
        self.actuator_range_rad = (
            self.get_parameter('actuator_range_rad').get_parameter_value().double_value
        )

        # ── Puerto serie ──────────────────────────────────────────────────────
        self.ser = None
        if SERIAL_AVAILABLE:
            try:
                self.ser = serial.Serial(port, baudrate=baud, timeout=0.05)
                self.get_logger().info(
                    f'STM32 conectado en {port} @ {baud} bps'
                )
            except serial.SerialException as e:
                self.get_logger().error(
                    f'No se pudo abrir el puerto {port}: {e}\n'
                    'El nodo continuará sin hardware (modo simulación).'
                )
        else:
            self.get_logger().warn(
                'pyserial no está instalado. Ejecutando sin hardware real.\n'
                'Instalar con: pip3 install pyserial'
            )

        # ── Nombres de joints (orden fijo, mismo que URDF) ───────────────────
        self.joint_names = [
            'bracket_joint',        # base — motor DC + encoder
            'humerus_low_joint',    # actuador lineal 1 — potenciómetro
            'forearm_low_joint',    # actuador lineal 2 — potenciómetro
            'ubracket_joint',       # muñeca pitch — diferencial (W1+W2)/2
            'endeffector_joint',    # muñeca roll  — diferencial (W1-W2)/2
        ]

        # ── Límites de joints del URDF ────────────────────────────────────────
        self.joint_limits = {
            'bracket_joint':     (-1.5,  1.5),
            'humerus_low_joint': (-1.5,  1.5),
            'forearm_low_joint': (-1.5,  1.5),
            'ubracket_joint':    (-0.8,  0.8),
            'endeffector_joint': (-4.5,  4.5),
        }

        # ── Estado interno (protegido por lock) ───────────────────────────────
        self._lock = threading.Lock()
        self.hw_positions = {n: 0.0 for n in self.joint_names}
        self.cmd_positions = {n: 0.0 for n in self.joint_names}
        self._joint_index_map = {n: None for n in self.joint_names}

        # ── Publishers ────────────────────────────────────────────────────────
        # Posiciones reales del hardware (desde encoders/potenciómetros)
        self.pub_hw_js = self.create_publisher(
            JointState, '/hardware/joint_states', 10
        )

        # Sincronizar hardware → simulación (opcional)
        if self.sync_to_sim:
            self.pub_sim_js = self.create_publisher(
                JointState, '/joint_states', 10
            )

        # ── Subscriber ────────────────────────────────────────────────────────
        # Comandos de posición (desde MoveIt, teleop, nodos de tareas, etc.)
        self.sub_cmd = self.create_subscription(
            JointState,
            '/joint_states_cmd',
            self._cmd_callback,
            10,
        )

        # ── Timer de feedback ─────────────────────────────────────────────────
        self._feedback_timer = self.create_timer(
            1.0 / feedback_rate, self._publish_feedback
        )

        # ── Hilo de lectura serie ─────────────────────────────────────────────
        self._stop_read = False
        self._read_thread = threading.Thread(
            target=self._serial_read_loop, daemon=True
        )
        self._read_thread.start()

        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════════════╗\n'
            '║       STM32 Hardware Bridge — INICIADO       ║\n'
            '╠══════════════════════════════════════════════╣\n'
            f'║  Puerto   : {port:<33}║\n'
            f'║  Baudrate : {baud:<33}║\n'
            f'║  Sync sim : {str(self.sync_to_sim):<33}║\n'
            f'║  Feedback : {feedback_rate:<30.1f} Hz ║\n'
            '╠══════════════════════════════════════════════╣\n'
            '║  Suscrito a : /joint_states_cmd              ║\n'
            '║  Publica en : /hardware/joint_states         ║\n'
            '╚══════════════════════════════════════════════╝'
        )

    # ── Callback de comandos ──────────────────────────────────────────────────

    def _cmd_callback(self, msg: JointState):
        """Recibe JointState de comando y lo envía al STM32."""
        # Construir mapa de índices la primera vez
        if any(v is None for v in self._joint_index_map.values()):
            for i, name in enumerate(msg.name):
                if name in self._joint_index_map:
                    self._joint_index_map[name] = i

        with self._lock:
            for name in self.joint_names:
                idx = self._joint_index_map.get(name)
                if idx is not None and idx < len(msg.position):
                    lo, hi = self.joint_limits[name]
                    self.cmd_positions[name] = max(lo, min(hi, msg.position[idx]))

        self._send_command()

    def _send_command(self):
        """
        Convierte posiciones de joints a protocolo STM32 y escribe en serie.

        Protocolo enviado:
          CMD:B:<deg>;W1:<deg>;W2:<deg>;A1:<pct>;A2:<pct>\\n
        """
        if self.ser is None or not self.ser.is_open:
            return

        with self._lock:
            b_rad  = self.cmd_positions['bracket_joint']
            wp_rad = self.cmd_positions['ubracket_joint']
            wr_rad = self.cmd_positions['endeffector_joint']
            a1_rad = self.cmd_positions['humerus_low_joint']
            a2_rad = self.cmd_positions['forearm_low_joint']

        b_deg = math.degrees(b_rad)
        wp_deg = math.degrees(wp_rad)
        wr_deg = math.degrees(wr_rad)

        # Cinemática inversa del diferencial: W1 = pitch + roll, W2 = pitch - roll
        wm1_deg = wp_deg + wr_deg
        wm2_deg = wp_deg - wr_deg

        # Actuadores lineales: radianes → porcentaje de extensión (0-100%)
        a1_pct = self._rad_to_pct(a1_rad)
        a2_pct = self._rad_to_pct(a2_rad)

        cmd = (
            f'CMD:B:{b_deg:.2f};'
            f'W1:{wm1_deg:.2f};W2:{wm2_deg:.2f};'
            f'A1:{a1_pct:.1f};A2:{a2_pct:.1f}\r\n'
        )

        try:
            self.ser.write(cmd.encode('ascii'))
            self.get_logger().debug(f'→ STM32: {cmd.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Error enviando comando al STM32: {e}')
            try:
                self.ser.close()
            except Exception:
                pass

    # ── Publicación de feedback ───────────────────────────────────────────────

    def _publish_feedback(self):
        """Publica /hardware/joint_states (y opcionalmente /joint_states)."""
        with self._lock:
            positions = [self.hw_positions[n] for n in self.joint_names]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = positions
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)

        self.pub_hw_js.publish(msg)

        if self.sync_to_sim:
            self.pub_sim_js.publish(msg)

    # ── Hilo de lectura serie ─────────────────────────────────────────────────

    def _serial_read_loop(self):
        """Lee líneas del STM32 de forma continua y parsea el feedback."""
        while not self._stop_read and rclpy.ok():
            if self.ser is None or not self.ser.is_open:
                time.sleep(0.1)
                continue
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode('ascii', errors='ignore').strip()
                if line:
                    self._parse_feedback(line)
            except (OSError, serial.SerialException) as e:
                self.get_logger().error(f'Error leyendo serie: {e}')
                time.sleep(0.5)
            except Exception:
                pass

    def _parse_feedback(self, line: str):
        """
        Parsea una línea de feedback del STM32.

        Formato esperado:
          FB:BE:<ticks>;W1:<ticks>;W2:<ticks>;P1:<adc>;P2:<adc>

        Campos:
          BE  — encoder base (ticks con signo)
          W1  — encoder motor muñeca 1 (ticks con signo)
          W2  — encoder motor muñeca 2 (ticks con signo)
          P1  — ADC potenciómetro actuador lineal 1 (0-4095)
          P2  — ADC potenciómetro actuador lineal 2 (0-4095)
        """
        if not line.startswith('FB:'):
            return

        try:
            data: dict = {}
            for part in line[3:].split(';'):
                if ':' in part:
                    key, val = part.split(':', 1)
                    data[key.strip()] = float(val.strip())

            be_ticks = data.get('BE', 0.0)
            w1_ticks = data.get('W1', 0.0)
            w2_ticks = data.get('W2', 0.0)
            p1_adc   = data.get('P1', float(self.pot_min))
            p2_adc   = data.get('P2', float(self.pot_min))

            # Encoders → radianes
            base_rad = self._ticks_to_rad(be_ticks, self.base_ticks_per_rev)
            w1_rad   = self._ticks_to_rad(w1_ticks, self.wrist_ticks_per_rev)
            w2_rad   = self._ticks_to_rad(w2_ticks, self.wrist_ticks_per_rev)

            # Cinemática directa del diferencial de la muñeca
            pitch_rad = (w1_rad + w2_rad) / 2.0
            roll_rad  = (w1_rad - w2_rad) / 2.0

            # Potenciómetros → radianes (actuadores lineales)
            a1_rad = self._pot_to_rad(p1_adc)
            a2_rad = self._pot_to_rad(p2_adc)

            with self._lock:
                self.hw_positions['bracket_joint']     = self._clamp(
                    base_rad,  'bracket_joint')
                self.hw_positions['humerus_low_joint'] = self._clamp(
                    a1_rad,    'humerus_low_joint')
                self.hw_positions['forearm_low_joint'] = self._clamp(
                    a2_rad,    'forearm_low_joint')
                self.hw_positions['ubracket_joint']    = self._clamp(
                    pitch_rad, 'ubracket_joint')
                self.hw_positions['endeffector_joint'] = self._clamp(
                    roll_rad,  'endeffector_joint')

            self.get_logger().debug(
                f'← STM32 feedback: '
                f'base={math.degrees(base_rad):.1f}° '
                f'pitch={math.degrees(pitch_rad):.1f}° '
                f'roll={math.degrees(roll_rad):.1f}° '
                f'a1={math.degrees(a1_rad):.1f}° '
                f'a2={math.degrees(a2_rad):.1f}°'
            )

        except (ValueError, KeyError) as e:
            self.get_logger().debug(f'Error parseando feedback "{line}": {e}')

    # ── Conversiones de sensores ──────────────────────────────────────────────

    def _ticks_to_rad(self, ticks: float, ticks_per_rev: int) -> float:
        """Convierte encoder ticks (con signo) a radianes."""
        if ticks_per_rev == 0:
            self.get_logger().warn(
                'ticks_per_rev es 0 — verificar parámetros de calibración'
            )
            return 0.0
        return (ticks / ticks_per_rev) * (2.0 * math.pi)

    def _pot_to_rad(self, adc_value: float) -> float:
        """
        Convierte valor ADC de potenciómetro a ángulo de joint en radianes.

        Mapeo lineal: [pot_min, pot_max] → [-range/2, +range/2]
        """
        span = self.pot_max - self.pot_min
        if span == 0:
            self.get_logger().warn(
                'pot_min == pot_max — verificar parámetros pot_min y pot_max'
            )
            return 0.0
        adc_clamped = max(float(self.pot_min), min(float(self.pot_max), adc_value))
        normalized = (adc_clamped - self.pot_min) / span   # [0.0, 1.0]
        return normalized * self.actuator_range_rad - (self.actuator_range_rad / 2.0)

    def _rad_to_pct(self, rad: float) -> float:
        """
        Convierte ángulo de joint a porcentaje de extensión del actuador (0-100%).

        Mapeo lineal: [-range/2, +range/2] → [0, 100]
        """
        half = self.actuator_range_rad / 2.0
        clamped = max(-half, min(half, rad))
        return ((clamped + half) / self.actuator_range_rad) * 100.0

    def _clamp(self, value: float, joint_name: str) -> float:
        """Limita un valor dentro de los límites del URDF para el joint dado."""
        lo, hi = self.joint_limits[joint_name]
        return max(lo, min(hi, value))

    # ── Ciclo de vida ─────────────────────────────────────────────────────────

    def destroy_node(self):
        self._stop_read = True
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Stm32HardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
