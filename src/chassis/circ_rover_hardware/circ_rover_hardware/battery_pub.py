#!/usr/bin/env python3
"""
battery_pub.py  ──  paquete sugerido: circ_rover_hardware  (OPCIONAL)

El HTML deja el panel de batería en SIMULACIÓN mientras nadie publique
/battery_state. Este nodo publica sensor_msgs/BatteryState para que ese panel
(porcentaje, voltaje, corriente) muestre datos reales.

En ROS 1 leías voltaje/corriente del ESP32 por serie (el header 'GPS'/'TEMP'
del message_config.yaml). Aquí dejé la lectura como un stub: cámbiala por tu
fuente real (otra línea CSV del STM32/ESP32, un INA226 por I2C, el ADC, etc.).

Mapeo con el HTML (window.RoverUI.live.battery):
    percentage  -> barra y % de batería   (BatteryState usa 0..1; el HTML lo
                                            escala a 0..100 automáticamente)
    voltage     -> "xx.x V"
    current     -> "x.x A"   (el HTML toma valor absoluto)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class BatteryPub(Node):
    def __init__(self):
        super().__init__('battery_pub')

        # ── Parámetros de la química de tu pack (AJUSTA a tu batería) ──
        self.declare_parameter('cells', 6)          # p.ej. 6S LiPo
        self.declare_parameter('v_full', 4.2)       # V por celda llena
        self.declare_parameter('v_empty', 3.3)      # V por celda vacía
        self.declare_parameter('rate_hz', 1.0)

        self.cells = self.get_parameter('cells').value
        self.v_full = self.get_parameter('v_full').value
        self.v_empty = self.get_parameter('v_empty').value
        rate = self.get_parameter('rate_hz').value

        self.pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info('battery_pub publicando en /battery_state')

    def read_source(self):
        """
        STUB: devuelve (voltage_total, current_A).
        Reemplaza por tu lectura real. Ejemplos:
          • INA226 por I2C
          • otra línea CSV de tu STM32 (header tipo 'BAT,voltage,current')
          • el stream de voltaje/corriente que ya tenías en el ESP32
        """
        voltage = self.cells * 3.9   # <-- DUMMY: cámbialo por tu medición
        current = 1.5                # <-- DUMMY
        return voltage, current

    def tick(self):
        voltage, current = self.read_source()

        # Porcentaje por voltaje (lineal simple; suficiente para el panel).
        v_cell = voltage / max(self.cells, 1)
        pct = (v_cell - self.v_empty) / (self.v_full - self.v_empty)
        pct = max(0.0, min(1.0, pct))

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(voltage)
        msg.current = float(current)          # negativa = descargando, si quieres
        msg.percentage = float(pct)           # 0..1  (el HTML lo escala a %)
        msg.present = True
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()