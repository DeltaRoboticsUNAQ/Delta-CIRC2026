#!/usr/bin/env python3
"""
tank_teleop.py - Teleoperacion tipo tanque por teclado para el rover CIRC2025.

Controla las orugas izquierda y derecha de forma independiente (skid-steer),
en RPM de rueda (misma unidad que el firmware). Convierte a Twist y publica en
/cmd_vel, que es lo que suscribe rover_base_node (hace la cinematica inversa y
saca SP:L,R al STM32).

Cuando el UnifiedSafetyController este listo (falta configurar la OAK), solo
cambia el destino a /cmd_vel_raw:
  ros2 run circ_chassis_teleop tank_teleop --ros-args -p cmd_vel_topic:=/cmd_vel_raw

Modelo "latcheado": cada tecla ajusta la velocidad objetivo de esa oruga en
pasos; la velocidad se MANTIENE hasta que la cambies. Un toque = un paso;
mantener pulsada la tecla = rampa (por el auto-repeat del teclado).
Un timer publica el Twist actual a ~20 Hz, por encima del failsafe de 0.5 s.

FRENO ACTIVO (tecla espacio): en vez de solo mandar 0 (que deja rodar libre las
ruedas mientras el PID desacelera), lee la velocidad MEDIDA por encoders desde
/wheel/odometry y comanda par en contra (plugging) proporcional a esa velocidad
hasta que las ruedas se detienen de verdad. Lazo cerrado -> se autoapaga al
llegar a ~0, no sobrepasa a reversa. brake_gain=0 lo vuelve un paro suave.

Nota: la conversion RPM<->Twist es sin perdida siempre que wheel_radius (0.10) y
track_width (0.845) coincidan con rover_base_node.

Uso (hoy, sin nodo de seguridad -> directo a rover_base_node):
  ros2 run circ_chassis_teleop tank_teleop
"""

import math
import sys
import time
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

HELP = """
============ TELEOP TANQUE - CIRC2025 ============

     ORUGA IZQUIERDA          ORUGA DERECHA
         [w]  +                   [o]  +
         [s]  -                   [l]  -

   [i] ambas +     [,] ambas -     [k] igualar (recto)
   [espacio] FRENO ACTIVO (frena y para de verdad)

   Limite RPM:  [1..9] = 20..180 RPM     [0] = 200 RPM
   [q] / Ctrl-C  salir

  Un toque = un paso. Mantener pulsado = rampa.
==================================================
"""


class TankTeleop(Node):
    def __init__(self):
        super().__init__('tank_teleop')

        self.cmd_topic  = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        self.odom_topic = self.declare_parameter('odom_topic', '/wheel/odometry').value
        self.track      = float(self.declare_parameter('track_width', 0.845).value)
        self.R          = float(self.declare_parameter('wheel_radius', 0.10).value)
        self.max_rpm    = float(self.declare_parameter('max_rpm', 60.0).value)
        self.step_rpm   = float(self.declare_parameter('step_rpm', 5.0).value)
        self.rate       = float(self.declare_parameter('publish_rate', 20.0).value)
        # Freno activo
        self.brake_gain    = float(self.declare_parameter('brake_gain', 1.5).value)
        self.brake_eps_rpm = float(self.declare_parameter('brake_eps_rpm', 2.0).value)
        self.brake_timeout = float(self.declare_parameter('brake_timeout', 0.8).value)

        self.mps_per_rpm = (2.0 * math.pi * self.R) / 60.0

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)

        self.vL = 0.0            # comando RPM oruga izquierda
        self.vR = 0.0            # comando RPM oruga derecha
        self.meas_vL = 0.0       # RPM MEDIDA (encoders)
        self.meas_vR = 0.0
        self.braking = False
        self.brake_t0 = 0.0

        self.get_logger().info(
            f"tank_teleop -> '{self.cmd_topic}' | odom '{self.odom_topic}' | "
            f"lim={self.max_rpm:.0f} RPM | paso={self.step_rpm:.0f} | {self.rate} Hz")

    def _odom_cb(self, msg: Odometry):
        # Reconstruye la velocidad medida por lado (recuperacion exacta):
        #   vL = v - w*track/2 ;  vR = v + w*track/2
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.meas_vL = (v - w * self.track / 2.0) / self.mps_per_rpm
        self.meas_vR = (v + w * self.track / 2.0) / self.mps_per_rpm

    def clamp(self, rpm):
        return max(-self.max_rpm, min(self.max_rpm, rpm))

    def publish_rpm(self, l_rpm, r_rpm):
        vL_mps = l_rpm * self.mps_per_rpm
        vR_mps = r_rpm * self.mps_per_rpm
        msg = Twist()
        msg.linear.x  = 0.5 * (vR_mps + vL_mps)
        msg.angular.z = (vR_mps - vL_mps) / self.track
        self.pub.publish(msg)

    def brake_step(self):
        """Un ciclo de freno activo. Devuelve True cuando termino."""
        stopped = (abs(self.meas_vL) < self.brake_eps_rpm and
                   abs(self.meas_vR) < self.brake_eps_rpm)
        timed_out = (time.monotonic() - self.brake_t0) > self.brake_timeout
        if stopped or timed_out:
            self.braking = False
            self.vL = 0.0
            self.vR = 0.0
            self.publish_rpm(0.0, 0.0)
            return True
        # Par en contra proporcional a la velocidad medida (plugging)
        cmd_l = self.clamp(-self.brake_gain * self.meas_vL)
        cmd_r = self.clamp(-self.brake_gain * self.meas_vR)
        self.publish_rpm(cmd_l, cmd_r)
        return False


def main():
    rclpy.init()
    node = TankTeleop()
    period = 1.0 / node.rate if node.rate > 0 else 0.05

    settings = termios.tcgetattr(sys.stdin)
    print(HELP)

    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            r, _, _ = select.select([sys.stdin], [], [], period)
            if r:
                k = sys.stdin.read(1)
                if k in ('q', '\x03'):                 # q o Ctrl-C
                    break
                elif k == ' ':                          # FRENO ACTIVO
                    node.braking = True
                    node.brake_t0 = time.monotonic()
                elif k in '0123456789':                 # limite de RPM
                    d = int(k)
                    node.max_rpm = 200.0 if d == 0 else d * 20.0
                    node.braking = False
                elif k == '[':
                    node.step_rpm = max(1.0, node.step_rpm - 1.0)
                elif k == ']':
                    node.step_rpm = min(50.0, node.step_rpm + 1.0)
                else:
                    # teclas de movimiento -> cancelan el freno
                    if k == 'w':
                        node.vL += node.step_rpm; node.braking = False
                    elif k == 's':
                        node.vL -= node.step_rpm; node.braking = False
                    elif k == 'o':
                        node.vR += node.step_rpm; node.braking = False
                    elif k == 'l':
                        node.vR -= node.step_rpm; node.braking = False
                    elif k == 'i':
                        node.vL += node.step_rpm; node.vR += node.step_rpm; node.braking = False
                    elif k == ',':
                        node.vL -= node.step_rpm; node.vR -= node.step_rpm; node.braking = False
                    elif k == 'k':
                        avg = 0.5 * (node.vL + node.vR)
                        node.vL = avg; node.vR = avg; node.braking = False

                node.vL = node.clamp(node.vL)
                node.vR = node.clamp(node.vR)

            # procesa odometria entrante (para el freno)
            rclpy.spin_once(node, timeout_sec=0.0)

            # publica
            if node.braking:
                node.brake_step()
                estado = "FRENANDO"
            else:
                node.publish_rpm(node.vL, node.vR)
                estado = "        "

            sys.stdout.write(
                f"\r  cmd izq={node.vL:+4.0f}  der={node.vR:+4.0f} RPM  | "
                f"lim={node.max_rpm:3.0f}  paso={node.step_rpm:2.0f}  | "
                f"med L={node.meas_vL:+4.0f} R={node.meas_vR:+4.0f}  {estado} ")
            sys.stdout.flush()

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        stop = Twist()
        for _ in range(10):
            node.pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()
        print("\n[tank_teleop] detenido, velocidades a cero.")


if __name__ == '__main__':
    main()