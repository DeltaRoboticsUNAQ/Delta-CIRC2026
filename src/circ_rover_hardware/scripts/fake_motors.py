#!/usr/bin/env python3
"""
fake_motors.py   ──  paquete: circ_rover_hardware

Simulador de motores + encoders para probar TODA la cadena sin hardware.
Se pone en el lugar de (wheel_bridge + STM32 + motores + encoders):

  skid_steer_base --> cmd_wheel/left,right  -->  [ fake_motors ]  -->  wheel_vel/left,right --> skid_steer_base

Modela cada lado como un motor de primer orden: la velocidad MEDIDA no salta al
setpoint, se le acerca con una constante de tiempo `tau` (inercia), y se le añade
un poco de ruido tipo encoder. Para el resto del sistema es idéntico al hardware.

Lo corres EN LUGAR del wheel_bridge. Con esto cierras el lazo:
  /cmd_vel -> ruedas (simuladas) -> odometría -> EKF (fusiona con la IMU real).
"""

import math
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class FakeMotors(Node):
    def __init__(self):
        super().__init__('fake_motors')

        # ── Parámetros del modelo ──
        self.declare_parameter('tau', 0.3)          # [s] constante de tiempo (inercia)
        self.declare_parameter('noise_std', 0.5)    # [RPM] ruido del "encoder"
        self.declare_parameter('max_rpm', 223.0)    # tope físico (no-load del 5203)
        self.declare_parameter('rate', 50.0)        # [Hz] frecuencia de simulación

        self.tau       = float(self.get_parameter('tau').value)
        self.noise_std = float(self.get_parameter('noise_std').value)
        self.max_rpm   = float(self.get_parameter('max_rpm').value)
        self.rate      = float(self.get_parameter('rate').value)

        # ── Estado: setpoint recibido y velocidad real simulada, por lado ──
        self.sp_l = self.sp_r = 0.0      # lo que pide skid_steer_base
        self.w_l  = self.w_r  = 0.0      # velocidad real (con inercia)

        # ── I/O: idéntica a la del wheel_bridge real ──
        self.create_subscription(Float64, 'cmd_wheel/left',
                                 lambda m: setattr(self, 'sp_l', m.data), 10)
        self.create_subscription(Float64, 'cmd_wheel/right',
                                 lambda m: setattr(self, 'sp_r', m.data), 10)

        self.vel_l_pub = self.create_publisher(Float64, 'wheel_vel/left', 10)
        self.vel_r_pub = self.create_publisher(Float64, 'wheel_vel/right', 10)

        self.create_timer(1.0 / self.rate, self.step)

        self.get_logger().info(
            f'fake_motors: tau={self.tau}s, ruido={self.noise_std}RPM, '
            f'max={self.max_rpm}RPM — simulando ruedas SIN hardware')

    def step(self):
        dt = 1.0 / self.rate
        # Respuesta de primer orden hacia el setpoint (inercia del motor)
        alpha = 1.0 - math.exp(-dt / self.tau)
        self.w_l += alpha * (self._clamp(self.sp_l) - self.w_l)
        self.w_r += alpha * (self._clamp(self.sp_r) - self.w_r)

        # Publica la velocidad "medida" con un poco de ruido de encoder
        ml = Float64(); ml.data = self.w_l + random.gauss(0.0, self.noise_std)
        mr = Float64(); mr.data = self.w_r + random.gauss(0.0, self.noise_std)
        self.vel_l_pub.publish(ml)
        self.vel_r_pub.publish(mr)

    def _clamp(self, rpm):
        return max(-self.max_rpm, min(self.max_rpm, rpm))


def main(args=None):
    rclpy.init(args=args)
    node = FakeMotors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()