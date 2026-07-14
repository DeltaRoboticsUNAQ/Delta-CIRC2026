#!/usr/bin/env python3
"""
xbox_tank_teleop.py   ──  paquete: circ_chassis_teleop

Teleoperación TIPO TANQUE con control de Xbox para el rover skid-steer.
Nodo ROS2 NATIVO (rclpy) — publica directo a /cmd_vel, SIN rosbridge.

  CONTROL TANQUE PURO:
    Stick IZQUIERDO (eje Y)  -> velocidad del lado IZQUIERDO  [m/s]
    Stick DERECHO   (eje Y)  -> velocidad del lado DERECHO    [m/s]
  Se mezclan a Twist para /cmd_vel:
    linear.x  = (v_izq + v_der) / 2        (avance)
    angular.z = (v_der - v_izq) / track    (giro)

  BOTÓN B  -> STOP TOTAL (manda cero y se ignoran los sticks hasta soltar B)

  Basado en el script de Xbox de Sebastián López Tena, portado a rclpy nativo
  (sin roslibpy) y corregidos los bugs de la version original.

Requisitos:  sudo apt install python3-pygame   (o: pip install pygame)
El control se conecta por Bluetooth/USB; pygame lo ve como joystick.
"""

import math
import sys

import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ─────────── AJUSTES ───────────
MAX_SPEED   = 0.8      # [m/s] velocidad de cada lado al fondo del stick (RÁPIDA)
TRACK_WIDTH = 0.845    # [m] vía izq<->der (igual que rover_base.yaml)
DEADZONE    = 0.12     # zona muerta de los sticks (evita drift)
PUBLISH_HZ  = 30.0     # frecuencia de publicación de /cmd_vel

# Ejes en el mapeo típico de Xbox (Linux/pygame):
#   axis 1 = stick IZQUIERDO vertical   (arriba = negativo)
#   axis 4 = stick DERECHO vertical     (arriba = negativo)
AXIS_LEFT_Y  = 1
AXIS_RIGHT_Y = 4

# Botones (mapeo típico Xbox en pygame). B suele ser el índice 1.
BUTTON_B = 1


class XboxTankTeleop(Node):
    def __init__(self, joystick):
        super().__init__('xbox_tank_teleop')
        self.js = joystick
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.estop = False   # True mientras B esté presionado
        self.create_timer(1.0 / PUBLISH_HZ, self.loop)

        self.get_logger().info(
            f'Xbox TANQUE listo | max={MAX_SPEED} m/s por lado | '
            f'vía={TRACK_WIDTH} m | B = STOP TOTAL')

    @staticmethod
    def _deadzone(v):
        return 0.0 if abs(v) < DEADZONE else v

    def loop(self):
        # Procesar eventos de pygame (necesario para refrescar el estado)
        pygame.event.pump()

        # ── Botón B: stop total ──
        if self.js.get_button(BUTTON_B):
            if not self.estop:
                self.get_logger().warn('STOP TOTAL (boton B)')
            self.estop = True
            self._publish(0.0, 0.0)
            return
        else:
            self.estop = False

        # ── Lectura de sticks (tanque puro) ──
        # En Xbox, arriba del stick da valor NEGATIVO -> invertimos el signo
        raw_l = -self.js.get_axis(AXIS_LEFT_Y)
        raw_r = -self.js.get_axis(AXIS_RIGHT_Y)

        v_left  = self._deadzone(raw_l) * MAX_SPEED   # [m/s] lado izquierdo
        v_right = self._deadzone(raw_r) * MAX_SPEED   # [m/s] lado derecho

        # ── Mezcla tanque -> Twist ──
        linear_x  = (v_right + v_left) / 2.0
        angular_z = (v_right - v_left) / TRACK_WIDTH

        self._publish(linear_x, angular_z)

    def _publish(self, lin, ang):
        msg = Twist()
        msg.linear.x  = float(lin)
        msg.angular.z = float(ang)
        self.pub.publish(msg)


def init_joystick():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No se detecto ningun control. Conecta el Xbox (BT/USB) y reintenta.")
        pygame.quit()
        sys.exit(1)
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Usando control: {js.get_name()}")
    print(f"  Ejes: {js.get_numaxes()}  Botones: {js.get_numbuttons()}")
    print("  Stick IZQ = lado izquierdo | Stick DER = lado derecho | B = STOP")
    return js


def main(args=None):
    rclpy.init(args=args)
    js = init_joystick()
    node = XboxTankTeleop(js)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # frenar al salir
        try:
            node._publish(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        pygame.quit()


if __name__ == '__main__':
    main()