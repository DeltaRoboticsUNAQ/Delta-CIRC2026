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

        self.step = 0.01  # Sensibilidad ultra-fina para el modo manual
        self.joint_names = [
            'bracket_joint',
            'humerus_low_joint',
            'forearm_low_joint',
            'ubracket_joint',
            'endeffector_joint',
        ]

        # LÍMITES FÍSICOS
        self.limits = {
            'bracket_joint': (-1.39626, 1.39626),
            'humerus_low_joint': (-1.5, 0.0),
            'forearm_low_joint': (-1.216, 0.0),
            'ubracket_joint': (-0.785, 0.785),
            'endeffector_joint': (-3.1416, 3.1416),
        }

        # --- CONFIGURACIÓN DE PRESETS (ACTIONS) ---
        self.PRESET_MESA_BAJA = {
            'humerus_low_joint': -0.6,
            'forearm_low_joint': -0.4,
            'ubracket_joint': 0.0
        }
        self.PRESET_MESA_ALTA = {
            'humerus_low_joint': -1.2,
            'forearm_low_joint': -0.9,
            'ubracket_joint': -0.2
        }
        
        self.macro_running = False  # Bandera de seguridad para el golpecito

        self.positions: Dict[str, float] = {name: 0.0 for name in self.joint_names}
        self.initialized = False
        self.last_moved = 'bracket_joint'
        
        # Filtro para MoveIt
        self.last_sim_update = time.time()
        
        # Estado físico a enviar a la STM32
        self.h_pct = 0.0
        self.f_pct = 0.0
        self.b_vel = 0.0
        self.p_vel = 0.0
        self.r_vel = 0.0
        
        self.stop_timer = None

        # ROS2 Setup
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/arm/command', 10)
        self.action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

        # HID Setup
        self.joystick = hid.device()
        try:
            self.joystick.open(0x12bd, 0xa02f)
            self.joystick.set_nonblocking(True)
            self.get_logger().info('🕹️ Teleop HID Iniciado. Actions listas en botones 5, 6 y 7.')
        except Exception as e:
            self.get_logger().error(f'Error conectando: {e}')
            sys.exit(1)

        self.timer = self.create_timer(0.05, self.read_joystick_cb)

    def joint_state_cb(self, msg: JointState):
        if not self.initialized:
            for name in self.joint_names:
                if name in msg.name:
                    idx = msg.name.index(name)
                    self.positions[name] = msg.position[idx]
            self.initialized = True
            self.h_pct = self.calc_percentage('humerus_low_joint')
            self.f_pct = self.calc_percentage('forearm_low_joint')
            self.get_logger().info('✅ Inicialización completa.')

    def calc_percentage(self, joint_name):
        limit_rad = self.limits[joint_name][0]
        pct = (self.positions[joint_name] / limit_rad) * 100.0
        return max(0.0, min(100.0, pct))

    def clamp_limits(self):
        for n, (mn, mx) in self.limits.items():
            v = self.positions[n]
            if v < mn: v = mn
            if v > mx: v = mx
            self.positions[n] = v

    def publish_hardware_command(self):
        msg = Float64MultiArray()
        msg.data = [float(self.h_pct), float(self.f_pct), float(self.b_vel), float(self.p_vel), float(self.r_vel)]
        self.cmd_pub.publish(msg)

    def stop_roboclaws(self):
        self.b_vel = 0.0
        self.p_vel = 0.0
        self.r_vel = 0.0
        self.publish_hardware_command()

    # --- MACRO: EL GOLPECITO OPTIMIZADO (PÁJARO CARPINTERO) ---
    def ejecutar_golpecito_thread(self):
        self.macro_running = True
        print("🔨 [ACTION] ¡Pájaro carpintero activo! Estirando actuadores...")
        
        # Guardamos la posición original de ambos eslabones
        pos_original_h = self.positions['humerus_low_joint']
        pos_original_f = self.positions['forearm_low_joint']
        
        # Golpe de penetración: Ambos se estiran (restamos radianes) con mayor amplitud (0.35)
        self.positions['humerus_low_joint'] -= 0.35  
        self.positions['forearm_low_joint'] -= 0.35  
        self.clamp_limits()
        
        # Recalculamos los porcentajes físicos reales
        self.h_pct = self.calc_percentage('humerus_low_joint')
        self.f_pct = self.calc_percentage('forearm_low_joint')
        
        # Enviamos la orden inmediata a simulación y hardware
        self.send_trajectory_action()
        self.publish_hardware_command()
        
        # Damos 0.6 segundos completos para que los actuadores venzan la inercia y se muevan
        time.sleep(0.6)  
        
        # Regresamos automáticamente a la posición original
        print("↩️ [ACTION] Retrayendo golpe a posición segura.")
        self.positions['humerus_low_joint'] = pos_original_h
        self.positions['forearm_low_joint'] = pos_original_f
        
        self.h_pct = self.calc_percentage('humerus_low_joint')
        self.f_pct = self.calc_percentage('forearm_low_joint')
        
        self.send_trajectory_action()
        self.publish_hardware_command()
        
        time.sleep(0.7)  # Cooldown de protección para el botón
        self.macro_running = False

    def read_joystick_cb(self):
        if not self.initialized or self.macro_running:
            return

        report = self.joystick.read(64)
        if not report:
            return

        # --- MÁSCARAS DE SEGURIDAD Y MODIFICADORES ---
        trigger = report[6] & 1  # E-Stop
        btn2 = report[6] & 2     # Home
        btn3 = report[6] & 4     # Humerus Mod
        btn4 = report[6] & 8     # Forearm Mod

        # --- MÁSCARAS DE ACTIONS ---
        btn5 = report[6] & 16    # Botón 5 -> Mesa Baja (0.5m)
        btn6 = report[6] & 32    # Botón 6 -> Mesa Alta (1.0m)
        btn7 = report[6] & 64    # Botón 7 -> Golpecito Percusivo

        # 1. PARO DE EMERGENCIA
        if trigger:
            print("🚨 [ESTOP] ¡GATILLO PRESIONADO! DETENIENDO TODO EL BRAZO 🚨")
            if self.stop_timer:
                self.stop_timer.cancel()
            self.stop_roboclaws()
            return  

        # 2. RUTINA HOME
        if btn2:
            print("🏠 [HOME] Regresando a origen (0 rad)...")
            for name in self.joint_names:
                self.positions[name] = 0.0
            self.b_vel = 0.0; self.p_vel = 0.0; self.r_vel = 0.0
            self.h_pct = self.calc_percentage('humerus_low_joint')
            self.f_pct = self.calc_percentage('forearm_low_joint')
            self.send_trajectory_action()
            self.publish_hardware_command()
            time.sleep(0.2)
            return

        # 3. EXECUCIÓN DE ACTIONS
        if btn5:  
            print("🍳 [ACTION] Llevando brazo a posición: Mesa Baja (0.5m)")
            for joint, rad in self.PRESET_MESA_BAJA.items():
                self.positions[joint] = rad
            self.last_moved = 'humerus_low_joint'
            self.clamp_limits()
            self.h_pct = self.calc_percentage('humerus_low_joint')
            self.f_pct = self.calc_percentage('forearm_low_joint')
            self.send_trajectory_action()
            self.publish_hardware_command()
            time.sleep(0.3)
            return

        if btn6:  
            print("🚀 [ACTION] Llevando brazo a posición: Mesa Alta (1.0m)")
            for joint, rad in self.PRESET_MESA_ALTA.items():
                self.positions[joint] = rad
            self.last_moved = 'humerus_low_joint'
            self.clamp_limits()
            self.h_pct = self.calc_percentage('humerus_low_joint')
            self.f_pct = self.calc_percentage('forearm_low_joint')
            self.send_trajectory_action()
            self.publish_hardware_command()
            time.sleep(0.3)
            return

        if btn7:  
            # Lanzamos la nueva macro síncrona en su hilo
            threading.Thread(target=self.ejecutar_golpecito_thread).start()
            return

        moved_sim = False
        moved_real = False

        lateral = report[0]
        frontal = report[1]
        twist = report[2]
        throttle = report[4]

        DEADZONE_LOW = 100
        DEADZONE_HIGH = 154

        # --- MOVIMIENTO DE EJES CONTINUOS ---
        if twist < DEADZONE_LOW:
            self.positions['bracket_joint'] += self.step
            self.b_vel = 60.0
            self.last_moved = 'bracket_joint'
            moved_sim = True; moved_real = True
        elif twist > DEADZONE_HIGH:
            self.positions['bracket_joint'] -= self.step
            self.b_vel = -60.0
            self.last_moved = 'bracket_joint'
            moved_sim = True; moved_real = True

        if lateral < DEADZONE_LOW: 
            self.positions['ubracket_joint'] -= self.step
            self.p_vel = -60.0
            self.last_moved = 'ubracket_joint'
            moved_sim = True; moved_real = True
        elif lateral > DEADZONE_HIGH: 
            self.positions['ubracket_joint'] += self.step
            self.p_vel = 60.0
            self.last_moved = 'ubracket_joint'
            moved_sim = True; moved_real = True

        # --- EJE FRONTAL ---
        if frontal < DEADZONE_LOW: 
            if btn3 and not btn4:
                self.positions['humerus_low_joint'] -= self.step
                self.last_moved = 'humerus_low_joint'
            elif btn4 and not btn3:
                self.positions['forearm_low_joint'] -= self.step
                self.last_moved = 'forearm_low_joint'
            else:
                self.positions['humerus_low_joint'] -= self.step
                self.positions['forearm_low_joint'] -= self.step
                self.last_moved = 'humerus_low_joint'
            moved_sim = True; moved_real = True
            
        elif frontal > DEADZONE_HIGH: 
            if btn3 and not btn4:
                self.positions['humerus_low_joint'] += self.step
                self.last_moved = 'humerus_low_joint'
            elif btn4 and not btn3:
                self.positions['forearm_low_joint'] += self.step
                self.last_moved = 'forearm_low_joint'
            else:
                self.positions['humerus_low_joint'] += self.step
                self.positions['forearm_low_joint'] += self.step
                self.last_moved = 'humerus_low_joint'
            moved_sim = True; moved_real = True

        # --- THROTTLE ---
        throttle_pct = (255 - throttle) / 255.0
        ee_min, ee_max = self.limits['endeffector_joint']
        ee_target = ee_min + (throttle_pct * (ee_max - ee_min))
        
        if abs(self.positions['endeffector_joint'] - ee_target) > 0.05:
            self.positions['endeffector_joint'] = ee_target
            self.last_moved = 'endeffector_joint'
            moved_sim = True

        # --- PROCESAMIENTO ---
        if moved_sim:
            current_time = time.time()
            self.clamp_limits()
            
            if current_time - self.last_sim_update > 0.1:
                self.h_pct = self.calc_percentage('humerus_low_joint')
                self.f_pct = self.calc_percentage('forearm_low_joint')
                self.send_trajectory_action()
                self.last_sim_update = current_time

        if moved_real:
            self.publish_hardware_command()
            if self.stop_timer:
                self.stop_timer.cancel()
            self.stop_timer = threading.Timer(0.15, self.stop_roboclaws)
            self.stop_timer.start()

    def send_trajectory_action(self):
        if not self.action_client.server_is_ready():
            return
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [self.positions[n] for n in self.joint_names]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 150000000  
        goal_msg.trajectory.points.append(point)
        self.action_client.send_goal_async(goal_msg)

        nombre = self.last_moved.split('_')[0].capitalize()
        if self.last_moved in ['humerus_low_joint', 'forearm_low_joint']:
            pct = self.h_pct if self.last_moved == 'humerus_low_joint' else self.f_pct
            print(f"🎯 Sim Goal -> ({nombre}: {round(pct)}%) | Rad: {round(self.positions[self.last_moved], 3)}")
        else:
            print(f"🎯 Sim Goal -> ({nombre}: {round(self.positions[self.last_moved], 3)} rad)")

    def destroy_node(self):
        if self.stop_timer:
            self.stop_timer.cancel()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArmHidTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass 
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()