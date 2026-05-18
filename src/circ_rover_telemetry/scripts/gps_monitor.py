#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class GPSMonitor(Node):
    def __init__(self):
        super().__init__('gps_monitor')
        
        # Suscriptores
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Variables de estado
        self.origin_lat = None
        self.origin_lon = None
        self.current_lat = None
        self.current_lon = None
        self.current_time_sec = 0
        self.current_time_nsec = 0
        self.distance = 0.0
        
        self.state = "Detenido"
        
        self.get_logger().info("Monitor GPS iniciado. Esperando datos de /gps/fix y /odom...")
        
        # Timer para imprimir el reporte en consola a 5 Hz (cada 0.2s) para lecturas más fluidas
        self.timer = self.create_timer(0.2, self.print_report)

    def gps_callback(self, msg):
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            if self.origin_lat is None:
                self.origin_lat = msg.latitude
                self.origin_lon = msg.longitude
                self.get_logger().info(f"==> ORIGEN GPS FIJADO EN Lat: {self.origin_lat:.8f}, Lon: {self.origin_lon:.8f} <==")
            
            self.current_lat = msg.latitude
            self.current_lon = msg.longitude
            self.current_time_sec = msg.header.stamp.sec
            self.current_time_nsec = msg.header.stamp.nanosec
            
            self.calculate_distance()

    def odom_callback(self, msg):
        # Determinamos la dirección asumiendo el eje X como frente (estándar diferencial)
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        
        # Tolerancias para ruido de simulador
        if abs(vx) < 0.01 and abs(wz) < 0.01:
            self.state = "Detenido 🛑"
        elif vx > 0.01 and abs(wz) < 0.1:
            self.state = "Avanzando Adelante ⬆️"
        elif vx < -0.01 and abs(wz) < 0.1:
            self.state = "Retrocediendo Atrás ⬇️"
        elif abs(vx) < 0.1 and wz > 0.01:
            self.state = "Girando Izquierda ⬅️"
        elif abs(vx) < 0.1 and wz < -0.01:
            self.state = "Girando Derecha ➡️"
        elif vx > 0.01 and wz > 0.01:
            self.state = "Curva Adelante Izquierda ↖️"
        elif vx > 0.01 and wz < -0.01:
            self.state = "Curva Adelante Derecha ↗️"
        elif vx < -0.01 and wz > 0.01:
            self.state = "Curva Atrás Izquierda ↙️"
        elif vx < -0.01 and wz < -0.01:
            self.state = "Curva Atrás Derecha ↘️"

    def calculate_distance(self):
        # Aproximación plana para distancias cortas (Haversine simple / Equirrectangular)
        # 1 grado de latitud ~= 111,320 metros
        lat_diff_m = (self.current_lat - self.origin_lat) * 111320.0
        
        # 1 grado de longitud ~= 111,320 metros * cos(latitud)
        lon_diff_m = (self.current_lon - self.origin_lon) * (111320.0 * math.cos(math.radians(self.origin_lat)))
        
        self.distance = math.sqrt(lat_diff_m**2 + lon_diff_m**2)

    def print_report(self):
        if self.origin_lat is not None:
            time_str = f"{self.current_time_sec}.{self.current_time_nsec:09d}"
            print(f"\033[K\r⏱️ Ts: {time_str} | 🌍 Lat: {self.current_lat:.7f}, Lon: {self.current_lon:.7f} | 📍 Dist: \033[92m{self.distance:.2f} m\033[0m | 🚀 \033[96m{self.state}\033[0m", end='', flush=True)

def main(args=None):
    rclpy.init(args=args)
    node = GPSMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nMonitor detenido.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
