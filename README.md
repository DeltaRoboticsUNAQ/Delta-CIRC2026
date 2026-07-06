# CIRC2025 – Chasis en ROS2 (Humble)

Este workspace contiene una migración **enfocada únicamente en el chasis** del rover a **ROS2 Humble**, diseñada para:
- Probarse **desde hoy sin IMU** (proximidad sectorizada, alertas acústicas “inteligentes”, AEB TTC, control de velocidad adaptativo parcial, UI de operador, Pure Pursuit con GPS).
- Quedar **lista para expansión** (IMU real, LiDAR, más cámaras, Nav2).

## Plan de actividades y roadmap

- Configuración inicial del workspace Nav2. Definición del modelo cinemático diferencial y nodos de teleoperación básicos.
- Desarrollo de nodos de hardware, E-stop (parada de emergencia por software) y configuración de sensores.
- Integración de IMU y Encoders. Fusión de odometría vía EKF.
- Ajuste de navegación autónoma. Implementación de GPS Waypoints y tracking para la prueba de "Exploration".
- Dashboard de control y monitor de vibración activo.
- Integración en entorno completo "Full Stack". Depuración y dry-runs en terreno irregular.
- Optimización, preparación de repuestos y ajustes en Utah (CIRC).

## Paquetes
- `circ_chassis_msgs`: mensajes/servicios del chasis (proximidad, AEB, control adaptativo, métricas IMU-preparadas).
- `circ_nav_msgs`: interfaces de navegación (acción de waypoints GPS + `GeoPoint` minimalista).
- `circ_chassis_core`: nodos ROS2 (proximidad, AEB, speed-adaptive, mux de cmd_vel, GPS-localization, Pure Pursuit, bridge serial, UI).
- `circ_chassis_bringup`: launch + YAML de parámetros.
- `circ_rover_description`: URDF/meshes del rover (reutilizado de `rover_simple_v1_2`).
- `circ_chassis_teleop`: teleoperación con joystick (`joy` + `teleop_twist_joy`) publicando a `/cmd_vel_teleop`.

## Dependencias externas para replicar el ws

Si vas a montar este workspace en otra máquina, aparte de ROS2 Humble base, estas son las dependencias extra que aparecen en el código y los launch:

- ROS 2 y navegación: launch, launch_ros, ament_index_python, rclpy, rclcpp, std_msgs, sensor_msgs, geometry_msgs, nav_msgs, std_srvs, tf2, tf2_ros, tf2_geometry_msgs, robot_localization, nav2_common, nav2_controller, nav2_planner, nav2_behaviors, nav2_bt_navigator, nav2_waypoint_follower, nav2_lifecycle_manager, nmea_navsat_driver.
- Hardware y teleoperación: joy, teleop_twist_joy, teleop_twist_keyboard, robot_state_publisher, joint_state_publisher_gui, rviz2, v4l2_camera.
- Visión y cámaras: cv_bridge, depthai_ros_driver, depthimage_to_laserscan, rosbridge_suite, web_video_server.
- Python y sistema: python3-serial, python3-opencv, jetson-stats (jtop).
- Diagnóstico y telemetría: diagnostic_msgs.
- Si quieres reproducir también los componentes fuente incluidos en este ws, no son paquetes de apt sino repos completos dentro de src: eProsima/Micro-XRCE-DDS-Client, micro_ros_setup, ros2/common_interfaces y uros.

La forma más cómoda de instalar la parte ROS en una máquina nueva es usar rosdep después de clonar el workspace:

rosdep install --from-paths src --ignore-src -r -y


## Quickstart (sin hardware, prueba inmediata)

1) Build:
```bash
source /opt/ros/humble/setup.bash
cd /home/stc/circ2025_migration/ros2_ws
colcon build
source install/setup.bash
```

Si venías de un build anterior sin `--merge-install` y ROS no encuentra paquetes, limpia y recompila:
```bash
rm -rf build install log
colcon build
source install/setup.bash
```

2) Levantar chasis en modo simulación de proximidad:
```bash
ros2 launch circ_chassis_bringup chassis.launch.py 
```

3) En otra terminal, teleoperar (publica a `/cmd_vel_teleop`):
```bash
source /opt/ros/humble/setup.bash
source /home/stc/circ2025_migration/ros2_ws/install/setup.bash
ros2 launch circ_chassis_teleop keyboard_teleop.launch.py
```

Nota: si no tienes instalado `teleop_twist_keyboard` en el sistema:
```bash
sudo apt install ros-humble-teleop-twist-keyboard
```
## Interfaces ROS2

### Topics principales
- Control:
  - `/cmd_vel_teleop` (in)
  - `/cmd_vel_nav` (in)
  - `/cmd_vel_raw` (mux)
  - `/cmd_vel_safe` (salida final hacia HW)

- Proximidad:
  - `/scan` (in, `sensor_msgs/LaserScan`)
  - `/chassis/proximity/telemetry` (out, `circ_chassis_msgs/ProximityTelemetry`)
  - `/chassis/proximity/acoustic_alert` (out, `circ_chassis_msgs/AcousticAlert`)

- Seguridad:
  - `/chassis/safety/aeb_status` (out)
  - `/chassis/safety/adaptive_speed_status` (out)

- GPS:
  - `/gps/fix` (in, `sensor_msgs/NavSatFix`)
  - `/gps/origin` (out, `circ_nav_msgs/GeoPoint`)
  - `/gps/odom` (out, `nav_msgs/Odometry`)

- IMU (preparado):
  - `/imu/data_raw` (in, `sensor_msgs/Imu`)
  - `/chassis/imu/vibration_metrics` (out)
  - `/chassis/imu/terrain_metrics` (out)

### Service
- `/chassis/set_autonomy_mode` (`circ_chassis_msgs/srv/SetAutonomyMode`)

### Action
- `/chassis/follow_waypoints_gps` (`circ_nav_msgs/action/FollowWaypointsGps`)

### TF
- `map → base_link` publicado por `gps_localization`.


cd ~/circ2025_migration/ros2_ws
colcon build --packages-select circ_rover_description
source install/setup.bash
ros2 launch circ_rover_description sim.launch.py


# 1. Lanza todo con hardware real
ros2 launch circ_rover_navigation bringup.launch.py use_sim:=false gps:=true

# 2. En otra terminal, corre el nodo de calibración
ros2 run circ_rover_navigation heading_calibration

# 3. Asegúrate de tener ESPACIO LIBRE al frente (va a avanzar ~3 m) y dispara:
ros2 service call /calibrate_heading std_srvs/srv/Trigger


Launch cabron
ros2 launch circ_rover_navigation bringup.launch.py use_sim:=true gps:=true gps_port:=/dev/ttyACMx
