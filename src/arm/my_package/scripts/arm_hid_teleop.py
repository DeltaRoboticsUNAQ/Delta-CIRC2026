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

        # PRESETS
        self.PRESET_MESA_ALTA = {'humerus_low_joint': -0.6, 'forearm_low_joint': -0.4, 'ubracket_joint': 0.0}
        self.PRESET_XLR = {'humerus_low_joint': 0.0, 'forearm_low_joint': 0.0, 'ubracket_joint': 0.0}

        self.macro_running = False
        self.positions: Dict[str, float] = {name: 0.0 for name in self.joint_names}
        self.initialized = False
        self.last_moved = 'bracket_joint'
        self.last_sim_update = time.time()

        self.cmd_mode = 0.0  
        self.val1 = 0.0
        self.val2 = 0.0
        self.b_vel = 0.0
        self.p_vel = 0.0
        self.r_vel = 0.0
        self.c_vel = 0.0

        self.stop_timer = None
        self.is_manual_moving = False

        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/arm/command', 10)
        self.action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

        self.joystick = hid.device()
        try:
            self.joystick.open(0x12bd, 0xa02f)
            self.joystick.set_nonblocking(True)
            self.get_logger().info('🕹️ Teleop Híbrido Listo. MANTÉN BTN 5 para controlar la muñeca.')
        except Exception as e:
            self.get_logger().error(f'Error HID: {e}')
            sys.exit(1)

        self.timer = self.create_timer(0.05, self.read_joystick_cb)
        self.init_warn_timer = self.create_timer(3.0, self._check_init)

    def _check_init(self):
        if self.initialized:
            self.init_warn_timer.cancel()
            return
        self.get_logger().warn('⏳ Esperando datos en /joint_states...')

    def joint_state_cb(self, msg: JointState):
        if not self.initialized:
            for name in self.joint_names:
                if name in msg.name:
                    idx = msg.name.index(name)
                    self.positions[name] = msg.position[idx]
            self.initialized = True
            
            self.cmd_mode = 1.0
            self.val1 = 0.0
            self.val2 = 0.0
            
            self.publish_hardware_command()

    def calc_percentage(self, joint_name):
        limit_min = self.limits[joint_name][0]
        limit_max = self.limits[joint_name][1]
        rango = limit_max - limit_min
        if rango == 0.0: return 0.0
        
        current_rad = self.positions[joint_name]
        pct = ((current_rad - limit_min) / rango) * 100.0
        return max(0.0, min(100.0, pct))

    def clamp_limits(self):
        for n, (mn, mx) in self.limits.items():
            self.positions[n] = max(mn, min(mx, self.positions[n]))

    def publish_hardware_command(self):
        msg = Float64MultiArray()
        # Ahora mandamos 7 datos en el arreglo
        msg.data = [float(self.cmd_mode), float(self.val1), float(self.val2), float(self.b_vel), float(self.p_vel), float(self.r_vel), float(self.c_vel)]
        self.cmd_pub.publish(msg)

    def stop_roboclaws(self):
        self.b_vel = 0.0; self.p_vel = 0.0; self.r_vel = 0.0; self.c_vel = 0.0
        if self.cmd_mode == 1.0:
            self.val1 = 0.0; self.val2 = 0.0
        else:
            self.val1 = self.calc_percentage('humerus_low_joint')
            self.val2 = self.calc_percentage('forearm_low_joint')
        self.publish_hardware_command()

    def read_joystick_cb(self):
        if not self.initialized or self.macro_running: return
        report = self.joystick.read(64)
        if not report: return

        trigger = report[6] & 1; btn2 = report[6] & 2; btn3 = report[6] & 4; btn4 = report[6] & 8
        btn5 = report[6] & 16; btn6 = report[6] & 32; btn7 = report[6] & 64; btn8 = report[6] & 128

        if trigger:
            if self.stop_timer: self.stop_timer.cancel()
            self.cmd_mode = 1.0
            self.stop_roboclaws()
            self.is_manual_moving = False
            return

        if btn2:
            self.cmd_mode = 0.0
            for name in self.joint_names:
                self.positions[name] = self.limits[name][0]
            self.positions['bracket_joint'] = 0.0
            self.positions['ubracket_joint'] = 0.0
            self.positions['endeffector_joint'] = 0.0
            
            self.b_vel = 0.0; self.p_vel = 0.0; self.r_vel = 0.0
            self.val1 = self.calc_percentage('humerus_low_joint')
            self.val2 = self.calc_percentage('forearm_low_joint')
            self.send_trajectory_action()
            self.publish_hardware_command()
            time.sleep(0.2)
            return

        # BTN 8 (Presets) - Le quitamos el btn6 porque ahora es para la garra
        if btn8:
            self.cmd_mode = 0.0  
            target = self.PRESET_XLR
            for joint, rad in target.items(): self.positions[joint] = rad
            self.clamp_limits()
            self.val1 = self.calc_percentage('humerus_low_joint')
            self.val2 = self.calc_percentage('forearm_low_joint')
            self.send_trajectory_action()
            self.publish_hardware_command()
            time.sleep(0.3)
            return

        moved_sim = False
        moved_real = False

        lateral = report[0]; frontal = report[1]; twist = report[2]
        DEADZONE_LOW = 100; DEADZONE_HIGH = 154

        lateral_vel = self.map_analog_to_vel(lateral, DEADZONE_LOW, DEADZONE_HIGH)
        frontal_vel = -self.map_analog_to_vel(frontal, DEADZONE_LOW, DEADZONE_HIGH)

        # ==========================================
        # 🦀 MODO GARRA (BOTÓN 6 MANTENIDO)
        # ==========================================
        if btn6:
            self.cmd_mode = 1.0
            
            # GOBERNADOR DE GARRA (Si cierra muy duro, bájale a 0.30)
            LIMITADOR_GARRA = 0.60 
            
            if abs(frontal_vel) > 0:
                self.c_vel = frontal_vel * LIMITADOR_GARRA
                moved_real = True
            else:
                self.c_vel = 0.0
            
            # Congelamos base, actuadores y muñeca por seguridad
            self.b_vel = 0.0; self.p_vel = 0.0; self.r_vel = 0.0
            self.val1 = 0.0; self.val2 = 0.0

        # ==========================================
        # 🎯 MODO MUÑECA (BOTÓN 5 MANTENIDO)
        # ==========================================
        elif btn5: # <--- ¡ELIF! AQUÍ ESTABA LA FUGA DE MOVIMIENTO
            self.cmd_mode = 1.0
            
            # GOBERNADORES (1.0 = 100%, 0.15 = 15%)
            LIMITADOR_PITCH = 0.15 
            LIMITADOR_ROLL  = 0.20 
            
            # ROTACIÓN DE MUÑECA (Roll) con el eje Lateral
            if abs(lateral_vel) > 0:
                self.r_vel = -lateral_vel * LIMITADOR_ROLL
                self.positions['endeffector_joint'] += (lateral_vel * 0.0002) * LIMITADOR_ROLL
                moved_sim = True; moved_real = True
            else:
                self.r_vel = 0.0

            # PITCHEO DE MUÑECA (Pitch) con el eje Frontal
            if abs(frontal_vel) > 0:
                self.p_vel = -frontal_vel * LIMITADOR_PITCH
                self.positions['ubracket_joint'] += (frontal_vel * 0.0002) * LIMITADOR_PITCH
                moved_sim = True; moved_real = True
            else:
                self.p_vel = 0.0
            
            # Congelamos base y actuadores por seguridad
            self.b_vel = 0.0
            self.val1 = 0.0
            self.val2 = 0.0

        # ==========================================
        # 🦾 MODO BRAZO NORMAL (BOTONES 5 Y 6 SUELTOS)
        # ==========================================
        else:
            self.p_vel = 0.0
            self.r_vel = 0.0

            # 🔥 GOBERNADOR DE LA BASE 🔥
            # Antes estaba clavado en 60.0. Ahorita lo bajamos a 25.0.
            # Juega con este número si lo sientes muy lento o todavía muy brusco.
            LIMITADOR_BASE = 25.0 

            # Twist -> Base
            if twist < DEADZONE_LOW:
                self.positions['bracket_joint'] += self.step
                self.b_vel = LIMITADOR_BASE
                moved_sim = True
                moved_real = True
            elif twist > DEADZONE_HIGH:
                self.positions['bracket_joint'] -= self.step
                self.b_vel = -LIMITADOR_BASE
                moved_sim = True
                moved_real = True
            else: 
                self.b_vel = 0.0

            # Frontal -> Actuadores Lineales
            if abs(frontal_vel) > 0:
                self.cmd_mode = 1.0  
                if btn3 and not btn4:
                    self.val1 = frontal_vel; self.val2 = 0.0; self.positions['humerus_low_joint'] += frontal_vel * 0.0002
                elif btn4 and not btn3:
                    self.val2 = frontal_vel; self.val1 = 0.0; self.positions['forearm_low_joint'] += frontal_vel * 0.0002
                else:
                    self.val1 = frontal_vel; self.val2 = frontal_vel
                    self.positions['humerus_low_joint'] += frontal_vel * 0.0002
                    self.positions['forearm_low_joint'] += frontal_vel * 0.0002
                moved_sim = True; moved_real = True

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