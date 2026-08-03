#!/usr/bin/env python3
import sys
import threading
import serial
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class LectorSTM32(Node):
    def __init__(self):
        super().__init__('arm_serial_bridge')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.02)
            self.get_logger().info(f'🚀 Puente Clásico Optimizado en {port}')
        except Exception as e:
            self.get_logger().error(f'❌ ERROR CRÍTICO: No se pudo abrir {port}: {e}')
            sys.exit(1)

        self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_sub = self.create_subscription(Float64MultiArray, '/arm/command', self.cmd_cb, 10)

        self._lock = threading.Lock()
        self._raw = {'a1': 0.0, 'a2': 0.0, 'base': 0.0, 'pitch': 0.0, 'roll': 0.0}
        self.running = True
        self.thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.thread.start()
        self.timer = self.create_timer(0.05, self.publish_joint_states)

    def cmd_cb(self, msg: Float64MultiArray):
        # Ahora esperamos 7 datos
        if len(msg.data) < 7: return

        c = int(msg.data[0])
        v1 = int(round(msg.data[1]))
        v2 = int(round(msg.data[2]))
        b = int(round(msg.data[3]))
        p = int(round(msg.data[4]))
        r = int(round(msg.data[5]))
        claw = int(round(msg.data[6])) # <--- NUEVO COMANDO DE GARRA

        # Tu Firewall de la muñeca sigue aquí intacto
        if self._raw['pitch'] < -440 and p < 0: 
            p = 0
            self.get_logger().warn("⚠️ LÍMITE SUPERIOR - Subida bloqueada")
            
        if self._raw['pitch'] > 545 and p > 0: 
            p = 0
            self.get_logger().warn("⚠️ LÍMITE INFERIOR - Bajada bloqueada")

        # Agregamos #CL a la trama serial
        if c == 0:
            trama = f"#A1,{v1}\n#A2,{v2}\n#BV,{b}\n#WP,{p}\n#WR,{r}\n#CL,{claw}\n"
        else:
            trama = f"#V1,{v1}\n#V2,{v2}\n#BV,{b}\n#WP,{p}\n#WR,{r}\n#CL,{claw}\n"

        try:
            self.ser.write(trama.encode('ascii'))
            self.ser.flush()
        except Exception: pass

    def reader_loop(self):
        buffer = b""
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    buffer += self.ser.read(128)
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        self.parse_line(line.decode('utf-8', errors='ignore').strip())
                else: time.sleep(0.01)
            except Exception: time.sleep(1)

    def parse_line(self, line):
        if not line.startswith('J,'): return
        parts = line.split(',')
        
        # Ahora esperamos 8 valores
        if len(parts) == 8:
            try:
                with self._lock:
                    self._raw['a1'] = float(parts[1])
                    self._raw['a2'] = float(parts[2])
                    self._raw['base'] = float(parts[3])
                    
                    enc_m1 = int(parts[6])
                    enc_m2 = int(parts[7])
                    
                    # Cinemática diferencial inversa
                    pitch_real = (enc_m1 + enc_m2) / 2.0
                    roll_real = (enc_m1 - enc_m2) / 2.0
                    
                    self._raw['pitch'] = pitch_real
                    self._raw['roll'] = roll_real
                    
                    # Usamos el Logger de ROS para forzar impresión en pantalla
                    self.get_logger().info(f"PITCH: {pitch_real:8.1f}  |  ROLL: {roll_real:8.1f}  ||  Raw M1: {enc_m1}, Raw M2: {enc_m2}")
            except ValueError: pass
        else:
            # Si por alguna razón llega con más o menos comas, que nos avise
            self.get_logger().warn(f"Llegaron {len(parts)} datos en vez de 8. Trama ignorada: {line}")

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = ['bracket_joint', 'humerus_low_joint', 'forearm_low_joint', 'ubracket_joint', 'endeffector_joint']

        with self._lock:
            # 0% físico = 0.0 rad (Contraído) | 100% físico = límite negativo (Estirado)
            humerus = (self._raw['a1'] / 100.0) * -1.32
            forearm = (self._raw['a2'] / 100.0) * -1.216

            base = self._raw['base'] * 0.0001
            
            # Nota: cuando calibremos, ajustaremos estos multiplicadores (0.01)
            ubracket = self._raw['pitch'] * 0.01
            endeffector = self._raw['roll'] * 0.01

        msg.position = [base, humerus, forearm, ubracket, endeffector]
        msg.velocity = [0.0]*5
        msg.effort = [0.0]*5
        self.js_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        if self.ser.is_open: self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LectorSTM32()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()