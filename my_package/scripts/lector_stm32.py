#!/usr/bin/env python3
import threading
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class ArmSerialBridge(Node):
    def __init__(self):
        super().__init__('arm_serial_bridge')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('publish_rate', 20.0)

        self.declare_parameter('act1_joint', 'humerus_low_joint')
        self.declare_parameter('act1_min', -1.32)
        self.declare_parameter('act1_max', 0.0)

        self.declare_parameter('act2_joint', 'forearm_low_joint')
        self.declare_parameter('act2_min', -1.216)
        self.declare_parameter('act2_max', 0.0)

        self.declare_parameter('base_joint', '')
        self.declare_parameter('base_scale', 0.0001)
        self.declare_parameter('pitch_joint', '')
        self.declare_parameter('pitch_scale', 0.0001)
        self.declare_parameter('roll_joint', '')
        self.declare_parameter('roll_scale', 0.0001)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)

        self._lock = threading.Lock()
        self._raw = {'a1': 0.0, 'a2': 0.0, 'base': 0.0, 'pitch': 0.0, 'roll': 0.0}

        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Conectado a STM32 en {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'No se pudo abrir {port}: {e}')

        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.cmd_sub = self.create_subscription(Float64MultiArray, 'arm/command', self.on_command, 10)

        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self.publish_joint_states)

        self._running = True
        if self.ser is not None:
            self._reader = threading.Thread(target=self.reader_loop, daemon=True)
            self._reader.start()

    def reader_loop(self):
        buf = b''
        while self._running and rclpy.ok():
            try:
                data = self.ser.read(128)
            except serial.SerialException:
                break
            if not data: continue
            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                self.parse_line(line.decode('utf-8', errors='ignore').strip())

    def parse_line(self, line):
        if not line.startswith('J,'): return
        parts = line.split(',')
        if len(parts) != 6: return
        try:
            with self._lock:
                self._raw['a1'] = float(parts[1])
                self._raw['a2'] = float(parts[2])
                self._raw['base'] = float(parts[3])
                self._raw['pitch'] = float(parts[4])
                self._raw['roll'] = float(parts[5])
        except ValueError:
            pass

    def on_command(self, msg):
        if self.ser is None: return
        d = list(msg.data)
        
        if len(d) < 6:
            self.get_logger().warn('arm/command requiere 6 valores: [mode, act1, act2, base, pitch, roll]')
            return
            
        mode, val1, val2, bvel, pvel, rvel = d[:6]
        
        if mode == 0.0:
            c1 = f'#V1,{int(round(self._clamp(val1, -100, 100)))}\n'
            c2 = f'#V2,{int(round(self._clamp(val2, -100, 100)))}\n'
        else:
            c1 = f'#A1,{int(round(self._clamp(val1, 0, 100)))}\n'
            c2 = f'#A2,{int(round(self._clamp(val2, 0, 100)))}\n'

        cmds = (
            c1 + c2 +
            f'#BV,{int(round(self._clamp(bvel, -100, 100)))}\n'
            f'#WP,{int(round(self._clamp(pvel, -100, 100)))}\n'
            f'#WR,{int(round(self._clamp(rvel, -100, 100)))}\n'
        )
        try:
            self.ser.write(cmds.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().error(f'Error serie: {e}')

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def publish_joint_states(self):
        with self._lock:
            raw = dict(self._raw)

        p = self.get_parameter
        names, positions = [], []

        # --- Fórmula restaurada para empatar con teleop ---
        n = p('act1_joint').value
        if n:
            jmin = p('act1_min').value
            names.append(n); positions.append((raw['a1'] / 100.0) * jmin)

        n = p('act2_joint').value
        if n:
            jmin = p('act2_min').value
            names.append(n); positions.append((raw['a2'] / 100.0) * jmin)

        for key, jname_param, scale_param in (('base', 'base_joint', 'base_scale'), ('pitch', 'pitch_joint', 'pitch_scale'), ('roll', 'roll_joint', 'roll_scale')):
            n = p(jname_param).value
            if n: names.append(n); positions.append(raw[key] * p(scale_param).value)

        if not names: return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self.js_pub.publish(msg)

    def destroy_node(self):
        self._running = False
        if self.ser is not None:
            try: self.ser.close()
            except Exception: pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArmSerialBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()