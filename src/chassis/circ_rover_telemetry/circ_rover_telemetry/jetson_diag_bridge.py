#!/usr/bin/env python3
"""
jetson_diag_bridge.py  ──  paquete sugerido: circ_rover_hardware  (corre en la Jetson)

Publica diagnostic_msgs/DiagnosticArray en /diagnostics con las temperaturas y
carga reales de la Jetson, leídas con jetson-stats (jtop). Es lo que hace que el
panel de sistema del HTML (CPU/GPU temp) deje de estar simulado.

El HTML busca, dentro de cada KeyValue, claves que CONTENGAN:
    "cpu" + "temp"   -> tel.cpu   (°C)
    "gpu" + "temp"   -> tel.gpu   (°C)
    "rssi"/"signal"  -> tel.rssi  (dBm)   (opcional; aquí no se publica)
    "latency"        -> tel.lat   (ms)    (opcional; aquí no se publica)

Instalación (en la Jetson):
    sudo pip3 install -U jetson-stats        # requiere reiniciar el servicio jtop
    # (jetson-stats expone el servicio 'jtop'; reinicia la Jetson tras instalar)

Uso:
    ros2 run circ_rover_hardware jetson_diag_bridge
    # o:  python3 jetson_diag_bridge.py
"""

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

try:
    from jtop import jtop
    HAVE_JTOP = True
except Exception:
    HAVE_JTOP = False


class JetsonDiagBridge(Node):
    def __init__(self):
        super().__init__('jetson_diag_bridge')
        self.declare_parameter('rate_hz', 1.0)
        rate = self.get_parameter('rate_hz').value

        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        if not HAVE_JTOP:
            self.get_logger().error(
                'jetson-stats (jtop) no está instalado. '
                'Instala con: sudo pip3 install -U jetson-stats y reinicia.')
            # Igual creamos el timer para no morir; publicará vacío.
            self.jetson = None
        else:
            self.jetson = jtop()
            self.jetson.start()

        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info('jetson_diag_bridge publicando en /diagnostics')

    def tick(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        st = DiagnosticStatus()
        st.name = 'jetson'
        st.hardware_id = 'jetson'
        st.level = DiagnosticStatus.OK

        kvs = []
        if self.jetson is not None and self.jetson.ok():
            temps = self.jetson.temperature or {}
            # Los nombres de zona varían por modelo (CPU, GPU, SOC0, thermal...).
            cpu_t = self._pick(temps, ['CPU', 'cpu'])
            gpu_t = self._pick(temps, ['GPU', 'gpu'])
            if cpu_t is not None:
                kvs.append(KeyValue(key='CPU Temp', value=f'{cpu_t:.1f}'))
            if gpu_t is not None:
                kvs.append(KeyValue(key='GPU Temp', value=f'{gpu_t:.1f}'))

            # Carga media de CPU (porcentaje), opcional
            try:
                cpu = self.jetson.cpu or {}
                total = cpu.get('total', {})
                if 'idle' in total:
                    kvs.append(KeyValue(key='CPU Load',
                                        value=f"{100.0 - float(total['idle']):.0f}"))
            except Exception:
                pass

        st.values = kvs
        st.message = 'jtop OK' if kvs else 'sin lecturas de jtop'
        arr.status = [st]
        self.pub.publish(arr)

    @staticmethod
    def _pick(temps, keys):
        for k in keys:
            for name, val in temps.items():
                if k.lower() in str(name).lower():
                    # jtop puede devolver dict {'temp': x} o float
                    if isinstance(val, dict):
                        return float(val.get('temp', val.get('online', 0)))
                    try:
                        return float(val)
                    except Exception:
                        continue
        return None

    def destroy_node(self):
        try:
            if self.jetson is not None:
                self.jetson.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JetsonDiagBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()