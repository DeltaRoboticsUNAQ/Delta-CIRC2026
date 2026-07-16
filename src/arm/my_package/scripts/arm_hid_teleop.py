#!/usr/bin/env python3
import sys
import threading
import time
from typing import Dict
import hid
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class ArmHidTeleop(Node):
    def __init__(self):
        super().__init__('arm_hid_teleop_node')

        self.step = 0.01
        self.joint_names = ['bracket_joint', 'humerus_low_joint', 'forearm_low_joint', 'ubracket_joint', 'endeffector_joint']

        self.limits = {
            'bracket_joint': (-1.39626, 1.39626),
            'humerus_low_joint': (-1.32, 0.0),
            'forearm_low_joint': (-1.216, 0.0),
            'ubracket_joint': (-0.785, 0.785),
            'endeffector_joint': (-3.1416, 3.1416),
        }

        # Presets de posición (0.0=arriba/retraído, negativos=abajo/estirado)
        self.PRESET_MESA_BAJA = {'humerus_low_joint': -1.2, 'forearm_low_joint': -0.9, 'ubracket_joint': -0.2}
        self.PRESET_MESA_ALTA = {'humerus_low_joint': -0.6, 'forearm_low_joint': -0.4, 'ubracket_joint': 0.0}
        self.PRESET_XLR = {'humerus_low_joint': -1.0, 'forearm_low_joint': -1.216, 'ubracket_joint': 0.0}

        self.macro_running = False
        self.positions: Dict[str, float] = {name: 0.0 for name in self.joint_names}
        self.initialized = False
        self.last_moved = 'bracket_joint'
        self.last_sim_update = time.time()

        # Datos unificados
        self.cmd_mode = 0.0  # 0.0 = Posición (Actions), 1.0 = Velocidad (Manual)
        self.val1 = 0.0
        self.val2 = 0.0
        self.b_vel = 0.0
        self.p_vel = 0.0
        self.r_vel = 0.0

        self.stop_timer = None
        self.is_manual_moving = False

        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/arm/command', 10)
        self.action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

        self.joystick = hid.device()
        try:
            self.joystick.open(0x12bd, 0xa02f)
            self.joystick.set_nonblocking(True)
            self.get_logger().info('🕹️ Teleop HID Híbrido Listo (Sin Retrasos).')
        except Exception as e:
            self.get_logger().error(f'Error HID: {e}')
            self.get_logger().error('Verifica con lsusb que el joystick sea 12bd:a02f y que tengas permisos (regla udev o sudo).')
            sys.exit(1)

        self.timer = self.create_timer(0.05, self.read_joystick_cb)

        # CAMBIO: watchdog de inicialización. Antes, si el arm_serial_bridge no
        # estaba corriendo (puerto equivocado, USB caído), nunca llegaba
        # /joint_states y este nodo ignoraba el joystick EN SILENCIO.
        self.init_warn_timer = self.create_timer(3.0, self._check_init)

    def _check_init(self):
        if self.initialized:
            self.init_warn_timer.cancel()
            return
        self.get_logger().warn('⏳ Sin datos en /joint_states: ¿está corriendo arm_serial_bridge (lector_stm32)? Sin eso el teleop no hace NADA.')

    def joint_state_cb(self, msg: JointState):
        if not self.initialized:
            for name in self.joint_names:
                if name in msg.name:
                    idx = msg.name.index(name)
                    self.positions[name] = msg.position[idx]
            self.initialized = True
            self.cmd_mode = 0.0
            self.val1 = self.calc_percentage('humerus_low_joint')
            self.val2 = self.calc_percentage('forearm_low_joint')
            self.publish_hardware_command()
            self.get_logger().info('✅ Posición inicial sincronizada desde /joint_states.')

    def calc_percentage(self, joint_name):
        limit_rad = self.limits[joint_name][0]
        if limit_rad == 0.0: return 0.0
        pct = (self.positions[joint_name] / limit_rad) * 100.0
        return max(0.0, min(100.0, pct))

    def clamp_limits(self):
        for n, (mn, mx) in self.limits.items():
            self.positions[n] = max(mn, min(mx, self.positions[n]))

    def publish_hardware_command(self):
        msg = Float64MultiArray()
        msg.data = [float(self.cmd_mode), float(self.val1), float(self.val2), float(self.b_vel), float(self.p_vel), float(self.r_vel)]
        self.cmd_pub.publish(msg)

    def stop_roboclaws(self):
        self.b_vel = 0.0; self.p_vel = 0.0; self.r_vel = 0.0
        if self.cmd_mode == 1.0:
            # Modo velocidad: V=0 -> el STM32 corta PWM y sincroniza su objetivo
            self.val1 = 0.0; self.val2 = 0.0
        else:
            # CAMBIO: en modo posición NO mandar 0 (antes esto enviaba #A1,0/#A2,0
            # al soltar el stick y el brazo se retraía solo al 0%). Se mantiene
            # el objetivo de posición actual.
            self.val1 = self.calc_percentage('humerus_low_joint')
            self.val2 = self.calc_percentage('forearm_low_joint')
        self.publish_hardware_command()

    def macro_sleep(self, seconds):
        steps = int(seconds / 0.1)
        for _ in range(steps):
            if not self.macro_running: return False
            time.sleep(0.1)
        return True

    def ejecutar_saludo_thread(self):
        self.macro_running = True
        self.cmd_mode = 0.0 # Modo Posición para la Action
        print("👋 [ACTION] Iniciando Saludo...")

        pos_original_h = self.positions['humerus_low_joint']
        pos_original_f = self.positions['forearm_low_joint']

        self.positions['humerus_low_joint'] = 0.0
        self.positions['forearm_low_joint'] = -1.216
        self.clamp_limits()

        self.val1 = self.calc_percentage('humerus_low_joint')
        self.val2 = self.calc_percentage('forearm_low_joint')
        self.last_moved = 'forearm_low_joint'
        self.send_trajectory_action()
        self.publish_hardware_command()
        if not self.macro_sleep(4.0): return

        # Movimiento de la base a velocidad aumentada (usando p_vel en lateral)
        print("🧭 [ACTION] Girando base...")
        self.p_vel = 100.0
        self.publish_hardware_command()
        if not self.macro_sleep(1.0): return

        self.p_vel = -100.0
        self.publish_hardware_command()
        if not self.macro_sleep(2.0): return

        self.p_vel = 100.0
        self.publish_hardware_command()
        if not self.macro_sleep(1.0): return

        self.p_vel = 0.0
        self.publish_hardware_command()

        print("↩️ [ACTION] Retrayendo saludo.")
        self.positions['humerus_low_joint'] = pos_original_h
        self.positions['forearm_low_joint'] = pos_original_f
        self.clamp_limits()
        self.val1 = self.calc_percentage('humerus_low_joint')
        self.val2 = self.calc_percentage('forearm_low_joint')
        self.last_moved = 'humerus_low_joint'
        self.send_trajectory_action()
        self.publish_hardware_command()
        self.macro_sleep(4.0)
        self.macro_running = False

    def read_joystick_cb(self):
        if not self.initialized or self.macro_running: return
        report = self.joystick.read(64)
        if not report: return

        trigger = report[6] & 1; btn2 = report[6] & 2; btn3 = report[6] & 4; btn4 = report[6] & 8
        btn5 = report[6] & 16; btn6 = report[6] & 32; btn7 = report[6] & 64; btn8 = report[6] & 128

        if trigger:
            print("🚨 [ESTOP] GATILLO PRESIONADO 🚨")
            self.macro_running = False
            if self.stop_timer: self.stop_timer.cancel()
            # CAMBIO: E-STOP real. Forzar modo velocidad con 0 hace que el STM32
            # corte PWM y sincronice el objetivo donde quedó (frena EN EL SITIO).
            # Antes, en modo posición, esto mandaba #A1,0/#A2,0 -> retraía el brazo.
            self.cmd_mode = 1.0
            self.stop_roboclaws()
            self.is_manual_moving = False
            return

        if btn2:
            print("🏠 [HOME]")
            self.cmd_mode = 0.0
            for name in self.joint_names: self.positions[name] = 0.0
            self.b_vel = 0.0; self.p_vel = 0.0; self.r_vel = 0.0
            self.val1 = self.calc_percentage('humerus_low_joint')
            self.val2 = self.calc_percentage('forearm_low_joint')
            self.send_trajectory_action()
            self.publish_hardware_command()
            time.sleep(0.2)
            return

        # EXECUCIÓN DE PRESETS ESTÁTICOS
        if btn5 or btn6 or btn8:
            self.cmd_mode = 0.0  # Forzar Modo Posición
            if btn5: target = self.PRESET_MESA_BAJA
            elif btn6: target = self.PRESET_MESA_ALTA
            elif btn8: target = self.PRESET_XLR

            for joint, rad in target.items(): self.positions[joint] = rad
            self.last_moved = 'humerus_low_joint'
            self.clamp_limits()
            self.val1 = self.calc_percentage('humerus_low_joint')
            self.val2 = self.calc_percentage('forearm_low_joint')
            self.send_trajectory_action()
            self.publish_hardware_command()
            time.sleep(0.3)
            return

        if btn7:
            threading.Thread(target=self.ejecutar_saludo_thread).start()
            return

        moved_sim = False
        moved_real = False

        lateral = report[0]; frontal = report[1]; twist = report[2]; throttle = report[4]
        DEADZONE_LOW = 100; DEADZONE_HIGH = 154

        if twist < DEADZONE_LOW:
            self.positions['bracket_joint'] += self.step; self.b_vel = 60.0; moved_sim = True; moved_real = True
        elif twist > DEADZONE_HIGH:
            self.positions['bracket_joint'] -= self.step; self.b_vel = -60.0; moved_sim = True; moved_real = True
        else: self.b_vel = 0.0

        if lateral < DEADZONE_LOW:
            self.positions['ubracket_joint'] -= self.step; self.p_vel = -100.0; moved_sim = True; moved_real = True
        elif lateral > DEADZONE_HIGH:
            self.positions['ubracket_joint'] += self.step; self.p_vel = 100.0; moved_sim = True; moved_real = True
        else: self.p_vel = 0.0

        frontal_vel = -self.map_analog_to_vel(frontal, DEADZONE_LOW, DEADZONE_HIGH)

        if abs(frontal_vel) > 0:
            self.cmd_mode = 1.0  # Activar Modo Velocidad Directa para el Joystick
            if btn3 and not btn4:
                self.val1 = frontal_vel; self.val2 = 0.0; self.positions['humerus_low_joint'] += frontal_vel * 0.0002
            elif btn4 and not btn3:
                self.val2 = frontal_vel; self.val1 = 0.0; self.positions['forearm_low_joint'] += frontal_vel * 0.0002
            else:
                self.val1 = frontal_vel; self.val2 = frontal_vel
                self.positions['humerus_low_joint'] += frontal_vel * 0.0002
                self.positions['forearm_low_joint'] += frontal_vel * 0.0002
            moved_sim = True; moved_real = True

        throttle_pct = (255 - throttle) / 255.0
        ee_min, ee_max = self.limits['endeffector_joint']
        ee_target = ee_min + (throttle_pct * (ee_max - ee_min))
        if abs(self.positions['endeffector_joint'] - ee_target) > 0.05:
            self.positions['endeffector_joint'] = ee_target; moved_sim = True

        if moved_sim:
            current_time = time.time()
            self.clamp_limits()
            if current_time - self.last_sim_update > 0.1:
                self.send_trajectory_action()
                self.last_sim_update = current_time

        if moved_real:
            self.is_manual_moving = True
            self.publish_hardware_command()
            if self.stop_timer: self.stop_timer.cancel()
            self.stop_timer = threading.Timer(0.15, self.stop_roboclaws)
            self.stop_timer.start()
        else:
            if self.is_manual_moving:
                self.stop_roboclaws()
                self.is_manual_moving = False

    def map_analog_to_vel(self, val, dead_low, dead_high):
        if val < dead_low:
            return ((dead_low - val) / float(dead_low)) * -100.0
        elif val > dead_high:
            return ((val - dead_high) / (255.0 - dead_high)) * 100.0
        return 0.0

    def send_trajectory_action(self):
        if not self.action_client.server_is_ready(): return
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [self.positions[n] for n in self.joint_names]
        point.time_from_start.sec = 0; point.time_from_start.nanosec = 150000000
        goal_msg.trajectory.points.append(point)
        self.action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmHidTeleop()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()