#!/usr/bin/env python3
import sys
import termios
import tty
import threading
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

def getch_blocking():
    """Lee una tecla de forma bloqueante en el terminal actual."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class ArmMoveItTeleopActions(Node):
    def __init__(self):
        super().__init__('arm_moveit_teleop_actions')

        self.step = 0.05  # ~3 grados por teclazo en la simulación
        self.joint_names = [
            'bracket_joint',
            'humerus_low_joint',
            'forearm_low_joint',
            'ubracket_joint',
            'endeffector_joint',
        ]

        # LÍMITES FÍSICOS (0.0 es Contraído, El negativo es 100% Estirado)
        self.limits = {
            'bracket_joint': (-1.39626, 1.39626),
            'humerus_low_joint': (-1.5, 0.0),
            'forearm_low_joint': (-1.216, 0.0),
            'ubracket_joint': (-0.785, 0.785),
            'endeffector_joint': (-3.1416, 3.1416),
        }

        self.positions: Dict[str, float] = {name: 0.0 for name in self.joint_names}
        self.initialized = False
        self.last_moved = 'bracket_joint'

        # Estado físico para enviar a la STM32
        self.h_pct = 0.0
        self.f_pct = 0.0
        self.b_vel = 0.0
        self.p_vel = 0.0
        self.r_vel = 0.0
        self.stop_timer = None

        # Suscriptores y Publicadores
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/arm/command', 10)
        self.action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

        # Hilo para capturar el teclado
        self._stop = False
        self._thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self._thread.start()

        self.print_help()

    def print_help(self):
        self.get_logger().info('========================================')
        self.get_logger().info('🎮 CONTROL HÍBRIDO (SIMULACIÓN + FÍSICO) 🎮')
        self.get_logger().info('----------------------------------------')
        self.get_logger().info('Teclas RoboClaw (Velocidad en hardware, pasos en sim):')
        self.get_logger().info('  A/D : Base (Izquierda / Derecha)')
        self.get_logger().info('  W/S : Muñeca Pitch (Subir / Bajar)')
        self.get_logger().info('  Q/E : Muñeca Roll (Rotar Izq / Der)')
        self.get_logger().info('----------------------------------------')
        self.get_logger().info('Teclas Actuadores (Posición rápida 0-100%):')
        self.get_logger().info('  1, 2, 3 : Humerus al 25%, 50%, 75%')
        self.get_logger().info('  4, 5, 6 : Forearm al 25%, 50%, 75%')
        self.get_logger().info('  0       : Ambos actuadores al 0% (Contraídos)')
        self.get_logger().info('  9       : Ambos actuadores al 100% (Estirados)')
        self.get_logger().info('----------------------------------------')
        self.get_logger().info('Teclas Actuadores (Paso a paso):')
        self.get_logger().info('  R/F : Humerus (+/-)')
        self.get_logger().info('  T/G : Forearm (+/-)')
        self.get_logger().info('  Ctrl+C  : Salir')
        self.get_logger().info('========================================')

    def joint_state_cb(self, msg: JointState):
        if not self.initialized:
            for name in self.joint_names:
                if name in msg.name:
                    idx = msg.name.index(name)
                    self.positions[name] = msg.position[idx]
            self.initialized = True
            
            # Sincronizar porcentajes iniciales
            self.h_pct = self.calc_percentage('humerus_low_joint')
            self.f_pct = self.calc_percentage('forearm_low_joint')
            self.get_logger().info('¡Posiciones sincronizadas! Listo para mover.')

    def calc_percentage(self, joint_name):
        """Calcula el % de extensión. 0.0 rad = 0%, límite negativo = 100%"""
        limit_rad = self.limits[joint_name][0]
        pct = (self.positions[joint_name] / limit_rad) * 100.0
        return max(0.0, min(100.0, pct))

    def set_percentage(self, joint_name, pct):
        """Traduce un porcentaje físico a la simulación en radianes."""
        limit_rad = self.limits[joint_name][0]
        self.positions[joint_name] = (pct / 100.0) * limit_rad
        self.last_moved = joint_name

    def clamp_limits(self):
        for n, (mn, mx) in self.limits.items():
            v = self.positions[n]
            if v < mn: v = mn
            if v > mx: v = mx
            self.positions[n] = v

    def keyboard_loop(self):
        try:
            while not self._stop and rclpy.ok():
                key = getch_blocking()
                self.handle_key(key)
        except Exception as e:
            pass  # Evitamos que ensucie la terminal al cerrar

    def publish_hardware_command(self):
        """Envía la trama al lector_stm32.py (y de ahí a la placa física)"""
        msg = Float64MultiArray()
        msg.data = [float(self.h_pct), float(self.f_pct), float(self.b_vel), float(self.p_vel), float(self.r_vel)]
        self.cmd_pub.publish(msg)

    def stop_roboclaws(self):
        """Detiene los motores RoboClaw después de un 'Jog'"""
        self.b_vel = 0.0
        self.p_vel = 0.0
        self.r_vel = 0.0
        self.publish_hardware_command()

    def trigger_roboclaw_stop(self):
        """Programa un auto-stop de 200ms para emular un pequeño paso en hardware"""
        if self.stop_timer is not None:
            self.stop_timer.cancel()
        self.stop_timer = threading.Timer(0.2, self.stop_roboclaws)
        self.stop_timer.start()

    def handle_key(self, key: str):
        if key in ('\x03',):  # Ctrl+C
            self._stop = True
            return

        if not self.initialized:
            self.get_logger().warn('Aún no recibo datos de RViz...')
            return

        moved_sim = False
        moved_real = False
        
        # --- RoboClaws (Motores continuos) ---
        if key in ('a', 'A'):
            self.positions['bracket_joint'] += self.step; self.last_moved = 'bracket_joint'
            self.b_vel = 60.0; moved_sim = True; moved_real = True
        elif key in ('d', 'D'):
            self.positions['bracket_joint'] -= self.step; self.last_moved = 'bracket_joint'
            self.b_vel = -60.0; moved_sim = True; moved_real = True
        elif key in ('w', 'W'):
            self.positions['ubracket_joint'] += self.step; self.last_moved = 'ubracket_joint'
            self.p_vel = 60.0; moved_sim = True; moved_real = True
        elif key in ('s', 'S'):
            self.positions['ubracket_joint'] -= self.step; self.last_moved = 'ubracket_joint'
            self.p_vel = -60.0; moved_sim = True; moved_real = True
        elif key in ('q', 'Q'):
            self.positions['endeffector_joint'] += self.step; self.last_moved = 'endeffector_joint'
            self.r_vel = 60.0; moved_sim = True; moved_real = True
        elif key in ('e', 'E'):
            self.positions['endeffector_joint'] -= self.step; self.last_moved = 'endeffector_joint'
            self.r_vel = -60.0; moved_sim = True; moved_real = True

        # --- Actuadores Lineales (Paso a paso) ---
        elif key in ('r', 'R'):
            self.positions['humerus_low_joint'] -= self.step; self.last_moved = 'humerus_low_joint'; moved_sim = True; moved_real = True
        elif key in ('f', 'F'):
            self.positions['humerus_low_joint'] += self.step; self.last_moved = 'humerus_low_joint'; moved_sim = True; moved_real = True
        elif key in ('t', 'T'):
            self.positions['forearm_low_joint'] -= self.step; self.last_moved = 'forearm_low_joint'; moved_sim = True; moved_real = True
        elif key in ('g', 'G'):
            self.positions['forearm_low_joint'] += self.step; self.last_moved = 'forearm_low_joint'; moved_sim = True; moved_real = True

        # --- Actuadores Lineales (Porcentajes directos) ---
        elif key == '1': self.set_percentage('humerus_low_joint', 25); moved_sim = True; moved_real = True
        elif key == '2': self.set_percentage('humerus_low_joint', 50); moved_sim = True; moved_real = True
        elif key == '3': self.set_percentage('humerus_low_joint', 75); moved_sim = True; moved_real = True
        elif key == '4': self.set_percentage('forearm_low_joint', 25); moved_sim = True; moved_real = True
        elif key == '5': self.set_percentage('forearm_low_joint', 50); moved_sim = True; moved_real = True
        elif key == '6': self.set_percentage('forearm_low_joint', 75); moved_sim = True; moved_real = True
        elif key == '0': 
            self.set_percentage('humerus_low_joint', 0)
            self.set_percentage('forearm_low_joint', 0)
            moved_sim = True; moved_real = True
        elif key == '9': 
            self.set_percentage('humerus_low_joint', 100)
            self.set_percentage('forearm_low_joint', 100)
            moved_sim = True; moved_real = True
        
        # Procesamiento
        if moved_sim:
            self.clamp_limits()
            # Recalcular porcentajes reales post-clamp
            self.h_pct = self.calc_percentage('humerus_low_joint')
            self.f_pct = self.calc_percentage('forearm_low_joint')
            self.send_trajectory_action()
            
        if moved_real:
            self.publish_hardware_command()
            # Si movimos un RoboClaw, programar freno automático
            if key in ('a','A','d','D','w','W','s','S','q','Q','e','E'):
                self.trigger_roboclaw_stop()

    def send_trajectory_action(self):
        if not self.action_client.server_is_ready():
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [self.positions[n] for n in self.joint_names]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000  

        goal_msg.trajectory.points.append(point)
        self.action_client.send_goal_async(goal_msg)
        
        # Consola limpia
        nombre = self.last_moved.split('_')[0].capitalize()
        if self.last_moved in ['humerus_low_joint', 'forearm_low_joint']:
            pct = self.h_pct if self.last_moved == 'humerus_low_joint' else self.f_pct
            print(f"🎯 Goal -> ({nombre}: {round(pct)}%) | Rad: {round(self.positions[self.last_moved], 3)}")
        else:
            print(f"🎯 Goal -> ({nombre}: {round(self.positions[self.last_moved], 3)} rad)")

    def destroy_node(self):
        self._stop = True
        if self.stop_timer:
            self.stop_timer.cancel()
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArmMoveItTeleopActions()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass # Silenciamos los errores de rclpy al apagar bruscamente
    finally:
        node.destroy_node()
        # Verificación segura para evitar el error de "already shutdown"
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()