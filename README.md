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

## Arquitectura (alto nivel)

```mermaid
flowchart LR
  Teleop[/cmd_vel_teleop/] --> Mux[cmd_vel_mux]
  Nav[/cmd_vel_nav/ Pure Pursuit] --> Mux
  Mux --> Raw[/cmd_vel_raw/]

  Scan[/scan/ LaserScan] --> Sector[proximity_sectorizer]
  Sector --> Tel[/chassis/proximity/telemetry/]
  Tel --> Acoustic[acoustic_alert]
  Tel --> AEB[aeb TTC]

  Tel --> Adapt[adaptive_speed]
  AEB --> Filter[cmd_vel_safety_filter]
  Adapt --> Filter
  Raw --> Filter
  Filter --> Safe[/cmd_vel_safe/]
  Safe --> HW[serial_bridge (opcional)]

  GPS[/gps/fix/] --> Loc[gps_localization]
  Loc --> Odom[/gps/odom/]
  Loc --> TF[(TF map→base_link)]
  Odom --> Nav

  Tel --> UI[operator_ui]
  AEB --> UI
  Adapt --> UI
  Acoustic --> UI
```

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

2) Levantar chasis en modo simulación de proximidad (sin serial):
```bash
ros2 launch circ_chassis_bringup chassis.launch.py use_scan_sim:=true use_serial:=false use_ui:=true
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

### Alternativa: Joystick
Si el operador usa joystick, lanza:
```bash
source /opt/ros/humble/setup.bash
source /home/stc/circ2025_migration/ros2_ws/install/setup.bash
ros2 launch circ_chassis_teleop joystick_teleop.launch.py
```
Config editable del mapeo: [src/circ_chassis_teleop/config/xbox_default.config.yaml](src/circ_chassis_teleop/config/xbox_default.config.yaml)

4) UI de operador:
- Abrir `http://localhost:8080/`

5) Probar AEB TTC:
- Baja `obstacle_distance_m` en `circ_chassis_bringup/config/chassis_params.yaml` o vía parámetros.
- Observa `/cmd_vel_safe` bajar a cero cuando el TTC cruza umbral.

## Operación con hardware (serial)

Lanza con serial habilitado:
```bash
ros2 launch circ_chassis_bringup chassis.launch.py use_scan_sim:=false use_serial:=true
```

Ajusta en `circ_chassis_bringup/config/chassis_params.yaml`:
- `serial_bridge.ros__parameters.port`
- `serial_bridge.ros__parameters.baudrate`
- `serial_bridge.ros__parameters.tx_header` (si tu MCU espera `TWIST,...`)
- `serial_bridge.ros__parameters.tx_mode`:
  - `twist_csv`: envía `v,w` (equivalente al ROS1 actual)
  - `skid_steer_lr_csv`: envía `v_left,v_right` (útil si tu firmware ya separa lados)
- `serial_bridge.ros__parameters.wheel_separation_m` (solo `skid_steer_lr_csv`)
- `serial_bridge.ros__parameters.linear_scale`, `angular_scale` (para convertir unidades/escala)

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

## Nota sobre GPS sin IMU
- `gps_localization` estima yaw con deltas de GPS (course over ground).
- A baja velocidad o con GPS ruidoso, el yaw es inestable; se recomienda:
  - `min_movement_for_heading_m` mayor (por ejemplo 1–2 m)
  - `heading_lpf_alpha` más bajo
  - Pure Pursuit con `lookahead_m` más grande.

### Timestamp de GPS
El `serial_bridge` acepta varias formas:
- `GPS,lat,lon`
- `GPS,lat,lon,unix_ms`
- `GPS,lat,lon,sec,nsec`
y publica `sensor_msgs/NavSatFix` en `/gps/fix`.

## Nota sobre alertas acústicas
La UI web reproduce beeps con WebAudio basados en `/chassis/proximity/acoustic_alert`.
Por restricciones de navegadores, debes presionar el botón **“Habilitar audio de alertas”** una vez.


cd ~/circ2025_migration/ros2_ws
colcon build --packages-select circ_rover_description
source install/setup.bash
ros2 launch circ_rover_description sim.launch.py

# WSL/Performance tip: run without heavy GUIs / LiDAR ray visualization
# (Defaults are unchanged; these are optional speed-focused overrides)
ros2 launch circ_rover_description sim.launch.py gui:=false rviz:=false lidar_visualize:=false


ros2 run circ_rover_safety aeb_controller --ros-args -p scan_topic:=/scan -p cmd_topic_in:=/cmd_vel_raw -p cmd_topic_out:=/cmd_vel

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel_raw
