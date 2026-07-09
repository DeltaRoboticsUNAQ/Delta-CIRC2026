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

        self.PRESET_MESA_BAJA = {'humerus_low_joint': -0.6, 'forearm_low_joint': -0.4, 'ubracket_joint': 0.0}
        self.PRESET_MESA_ALTA = {'humerus_low_joint': -1.2, 'forearm_low_joint': -0.9, 'ubracket_joint': -0.2}
        
        self.macro_running = False
        self.positions: Dict[str, float] = {name: 0.0 for name in self.joint_names}
        self.initialized = False
        
        # Variables de salida a STM32
        self.cmd_mode = 0.0 
        self.val1 = 0.0     
        self.val2 = 0.0
        self.b_vel = 0.0
        self.p_vel = 0.0
        self.r_vel = 0.0
        
        self.stop_timer = None
        self.real_positions = {name: 0.0 for name in self.joint_names}
        
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/arm/command', 10)
        self.action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

        self.joystick = hid.device()
        try:
            self.joystick.open(0x12bd, 0xa02f)
            self.joystick.set_nonblocking(True)
            self.get_logger().info('🕹️ Teleop HID Híbrido Iniciado.')
        except Exception as e:
            self.get_logger().error(f'Error conectando: {e}')
            sys.exit(1)

        self.timer = self.create_timer(0.05, self.read_joystick_cb)

    def joint_state_cb(self, msg: JointState):
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                self.real_positions[name] = msg.position[idx]

        if not self.initialized:
            for name in self.joint_names:
                self.positions[name] = self.real_positions.get(name, 0.0)
            self.initialized = True
            
            self.cmd_mode = 1.0 
            self.val1 = self.calc_percentage('humerus_low_joint')
            self.val2 = self.calc_percentage('forearm_low_joint')
            self.publish_hardware_command()

    # --- FÓRMULA ORIGINAL RESTAURADA ---
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
        self.cmd_mode = 0.0
        self.val1 = 0.0
        self.val2 = 0.0
        self.publish_hardware_command()
        
        if self.initialized:
            self.positions['humerus_low_joint'] = self.real_positions.get('humerus_low_joint', 0.0)
            self.positions['forearm_low_joint'] = self.real_positions.get('forearm_low_joint', 0.0)

    def map_analog_to_vel(self, val, dead_low, dead_high):
        if val < dead_low: return ((dead_low - val) / float(dead_low)) * -100.0
        elif val > dead_high: return ((val - dead_high) / (255.0 - dead_high)) * 100.0
        return 0.0

    # --- MACRO REPARADA (Ahora activa cmd_mode = 1.0) ---
    def ejecutar_golpecito_thread(self):
        self.macro_running = True
        self.cmd_mode = 1.0 
        print("🔨 [ACTION] ¡Pájaro carpintero activo! Estirando actuadores...")
        
        pos_original_h = self.positions['humerus_low_joint']
        pos_original_f = self.positions['forearm_low_joint']
        
        self.positions['humerus_low_joint'] -= 0.35  
        self.positions['forearm_low_joint'] -= 0.35  
        self.clamp_limits()
        
        self.val1 = self.calc_percentage('humerus_low_joint')
        self.val2 = self.calc_percentage('forearm_low_joint')
        
        self.send_trajectory_action()
        self.publish_hardware_command()
        time.sleep(0.6)  
        
        print("↩️ [ACTION] Retrayendo golpe a posición segura.")
        self.positions['humerus_low_joint'] = pos_original_h
        self.positions['forearm_low_joint'] = pos_original_f
        
        self.val1 = self.calc_percentage('humerus_low_joint')
        self.val2 = self.calc_percentage('forearm_low_joint')
        
        self.send_trajectory_action()
        self.publish_hardware_command()
        time.sleep(0.7)  
        self.macro_running = False

    def read_joystick_cb(self):
        if not self.initialized or self.macro_running: return
        report = self.joystick.read(64)
        if not report: return

        trigger = report[6] & 1; btn2 = report[6] & 2; btn3 = report[6] & 4; btn4 = report[6] & 8
        btn5 = report[6] & 16; btn6 = report[6] & 32; btn7 = report[6] & 64

        if trigger:
            if self.stop_timer: self.stop_timer.cancel()
            self.stop_roboclaws()
            return

        # ACCIONES (MODO POSICIÓN)
        if btn2 or btn5 or btn6:
            self.cmd_mode = 1.0 
            if btn2: target = {n: 0.0 for n in self.joint_names}
            elif btn5: target = self.PRESET_MESA_BAJA
            elif btn6: target = self.PRESET_MESA_ALTA
            
            for joint, rad in target.items():
                if joint in self.positions: self.positions[joint] = rad
            self.clamp_limits()
            
            self.b_vel = 0.0; self.p_vel = 0.0; self.r_vel = 0.0
            self.val1 = self.calc_percentage('humerus_low_joint')
            self.val2 = self.calc_percentage('forearm_low_joint')
            self.publish_hardware_command()
            self.send_trajectory_action()
            time.sleep(0.3)
            return
            
        if btn7:
            threading.Thread(target=self.ejecutar_golpecito_thread).start()
            return

        # MODO MANUAL (MODO VELOCIDAD DIRECTA)
        lateral = report[0]; frontal = report[1]; twist = report[2]; throttle = report[4]
        DEADZONE_LOW = 100; DEADZONE_HIGH = 154
        moved_real = False
        
        if twist < DEADZONE_LOW or twist > DEADZONE_HIGH:
            self.b_vel = self.map_analog_to_vel(twist, DEADZONE_LOW, DEADZONE_HIGH) * -0.6 # Invertido intencionalmente
            moved_real = True
        else: self.b_vel = 0.0

        if lateral < DEADZONE_LOW or lateral > DEADZONE_HIGH:
            self.p_vel = self.map_analog_to_vel(lateral, DEADZONE_LOW, DEADZONE_HIGH) * 0.6
            moved_real = True
        else: self.p_vel = 0.0

        # --- EJE INVERTIDO CON EL SIGNO MENOS (-) ---
        frontal_vel = -self.map_analog_to_vel(frontal, DEADZONE_LOW, DEADZONE_HIGH)
        
        self.cmd_mode = 0.0 
        self.val1 = 0.0
        self.val2 = 0.0
        
        if abs(frontal_vel) > 0:
            if btn3 and not btn4: self.val1 = frontal_vel
            elif btn4 and not btn3: self.val2 = frontal_vel
            else: self.val1 = frontal_vel; self.val2 = frontal_vel
            moved_real = True

        if moved_real:
            self.publish_hardware_command()
            if self.stop_timer: self.stop_timer.cancel()
            self.stop_timer = threading.Timer(0.15, self.stop_roboclaws)
            self.stop_timer.start()
        else:
            self.stop_roboclaws()

    def send_trajectory_action(self):
        if not self.action_client.server_is_ready(): return
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [self.positions[n] for n in self.joint_names]
        point.time_from_start.sec = 0; point.time_from_start.nanosec = 150000000
        goal_msg.trajectory.points.append(point)
        self.action_client.send_goal_async(goal_msg)

    def destroy_node(self):
        if self.stop_timer: self.stop_timer.cancel()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArmHidTeleop()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()