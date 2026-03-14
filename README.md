# Robotic Arm ROS2 Workspace

Workspace de ROS 2 para control y visualización de brazo robótico con STM32.

## 📋 Requisitos

- ROS 2 Humble (Ubuntu 22.04)
- Python 3.10+
- PySerial
- STM32 (recomendado: STM32F4 Nucleo) con firmware de control de motores

## 🏗️ Arquitectura del sistema

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          ROS 2 (PC / SBC)                               │
│                                                                         │
│  MoveIt / RViz  ──►  /joint_states_cmd  ──►  stm32_hardware_bridge     │
│                                                        │                │
│                                                    UART serie           │
│                                                    115200 bps           │
│                                                        │                │
│  /joint_states  ◄──  /hardware/joint_states  ◄────────┘                │
│  (simulación)        (posición real)                                    │
└─────────────────────────────────────────────────────────────────────────┘
                                  │ UART
┌─────────────────────────────────┼───────────────────────────────────────┐
│                          STM32  │                                        │
│                                 ▼                                       │
│  parse_command()  ──►  PID[base, W1, W2, A1, A2]  ──►  motor_set()    │
│                                                                         │
│  Sensores:                                                              │
│    TIM1 encoder  → Base (bracket_joint)                                 │
│    TIM3 encoder  → Muñeca motor 1 (W1)                                  │
│    TIM4 encoder  → Muñeca motor 2 (W2)                                  │
│    ADC ch0       → Potenciómetro actuador 1 (humerus_low_joint)         │
│    ADC ch1       → Potenciómetro actuador 2 (forearm_low_joint)         │
│                                                                         │
│  send_feedback()  ──►  FB:BE:<ticks>;W1:…;W2:…;P1:…;P2:…              │
└─────────────────────────────────────────────────────────────────────────┘
```

### Joints del brazo y hardware correspondiente

| Joint URDF | Actuador | Sensor |
|---|---|---|
| `bracket_joint` | Motor DC (base) | Encoder cuadratura |
| `humerus_low_joint` | Actuador lineal 1 | Potenciómetro (ADC 12-bit) |
| `forearm_low_joint` | Actuador lineal 2 | Potenciómetro (ADC 12-bit) |
| `ubracket_joint` | Diferencial muñeca (W1+W2)/2 | Encoder W1 + Encoder W2 |
| `endeffector_joint` | Diferencial muñeca (W1-W2)/2 | Encoder W1 + Encoder W2 |

### Cinemática diferencial de la muñeca

```
Lectura (STM32 → ROS 2):
  pitch_rad = (W1 + W2) / 2   →  ubracket_joint
  roll_rad  = (W1 - W2) / 2   →  endeffector_joint

Comando (ROS 2 → STM32):
  W1 = pitch + roll
  W2 = pitch - roll
```

### Nodos ROS 2

| Nodo | Tópico suscrito | Tópico publicado | Descripción |
|---|---|---|---|
| `robot_state_publisher` | — | `/tf`, `/robot_description` | Publica TF del modelo URDF |
| `stm32_hardware_bridge` | `/joint_states_cmd` | `/hardware/joint_states`, `/joint_states` | Puente ROS 2 ↔ STM32 |
| `arm_keyboard_teleop` | — | `/joint_states` | Teleop solo simulación |
| `arm_hw_teleop_bridge` | — | `/joint_states` | Teleop + envío directo a hardware |

## 🚀 Instalación

### En Ubuntu 22.04 / WSL2 / Máquina Virtual

1. **Instalar ROS 2 Humble** (si no está instalado):
```bash
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions
```

2. **Clonar este repositorio**:
```bash
mkdir -p ~/arm_ws/src
cd ~/arm_ws/src
git clone <TU_URL_DE_GITHUB> .
cd ..
```

3. **Instalar dependencias**:
```bash
sudo apt install python3-pip
pip3 install pyserial
sudo usermod -a -G dialout $USER  # Para acceso al puerto serie
```

4. **Compilar el workspace**:
```bash
cd ~/arm_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 🎮 Uso

### Simulación en RViz (sin hardware)
```bash
source install/setup.bash
ros2 launch my_package sim_rviz.launch.py
```

### Control con hardware STM32 (modo recomendado)
```bash
source install/setup.bash
ros2 launch my_package hardware.launch.py \
    port:=/dev/ttyACM0 \
    baudrate:=115200 \
    sync_to_sim:=true
```

El parámetro `sync_to_sim:=true` hace que las posiciones reales del hardware
(leídas desde encoders y potenciómetros) se reflejen en la simulación de RViz.

### Parámetros del launch de hardware

| Parámetro | Valor por defecto | Descripción |
|---|---|---|
| `port` | `/dev/ttyACM0` | Puerto serie del STM32 |
| `baudrate` | `115200` | Baudrate de comunicación |
| `sync_to_sim` | `true` | Feedback hardware → simulación |
| `feedback_rate_hz` | `50.0` | Frecuencia de publicación de feedback |
| `base_ticks_per_rev` | `2000` | Ticks encoder base por revolución |
| `wrist_ticks_per_rev` | `2000` | Ticks encoders muñeca por revolución |
| `pot_min` | `200` | ADC potenciómetro posición mínima |
| `pot_max` | `3800` | ADC potenciómetro posición máxima |

### Enviar comandos de posición al hardware
```bash
# Desde otro terminal (con el launch de hardware corriendo):
ros2 topic pub /joint_states_cmd sensor_msgs/msg/JointState \
    "{name: ['bracket_joint', 'humerus_low_joint', 'forearm_low_joint', \
              'ubracket_joint', 'endeffector_joint'], \
      position: [0.5, 0.3, -0.3, 0.2, 0.0]}"
```

### Monitorear feedback del hardware
```bash
ros2 topic echo /hardware/joint_states
```

### Simulación con MoveIt (sin hardware)
```bash
source install/setup.bash
ros2 launch arm_moveit_config demo.launch.py
```

## 🔌 Protocolo serie STM32

### ROS 2 → STM32 (comando):
```
CMD:B:<deg>;W1:<deg>;W2:<deg>;A1:<pct>;A2:<pct>\r\n
```

### STM32 → ROS 2 (feedback):
```
FB:BE:<ticks>;W1:<ticks>;W2:<ticks>;P1:<adc>;P2:<adc>\r\n
```

Ver `src/my_package/firmware/stm32_arm_firmware.md` para la referencia completa
del firmware STM32 (lectura de encoders, ADC, PID, control de motores).

## 🖥️ Configuración para WSL2

Si usas WSL2, los dispositivos USB necesitan mapearse:

### Método recomendado: usbipd-win

**En Windows PowerShell (como Administrador):**
```powershell
winget install --interactive --exact dorssel.usbipd-win
usbipd list
usbipd bind --busid X-X
usbipd attach --wsl --busid X-X
```

**En WSL:**
```bash
ls /dev/ttyACM* /dev/ttyUSB*  # Verificar dispositivo
```

## 📦 Estructura del Proyecto

```
arm_ws/
├── src/
│   ├── my_package/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── firmware/
│   │   │   └── stm32_arm_firmware.md   # Referencia firmware STM32
│   │   ├── launch/
│   │   │   ├── sim_rviz.launch.py      # Simulación sin hardware
│   │   │   ├── hardware.launch.py      # Control con STM32 real  ← NUEVO
│   │   │   └── sim_gazebo.launch.py    # Simulación Gazebo
│   │   ├── scripts/
│   │   │   ├── stm32_hardware_bridge.py   # Puente ROS2↔STM32  ← NUEVO
│   │   │   ├── arm_hardware_bridge.py     # Puente legado (Arduino)
│   │   │   ├── arm_hw_teleop_bridge.py    # Teleop + hardware
│   │   │   ├── arm_keyboard_teleop.py     # Teleop solo simulación
│   │   │   └── keyboard_arduino.py        # Teleop legado
│   │   └── urdf/
│   │       └── arm.xacro               # Modelo URDF del brazo
│   ├── arm_moveit_config/              # Configuración MoveIt 2
│   └── arm_tasks/                      # Tareas autónomas de alto nivel
├── build/
├── install/
└── log/
```

## 🐛 Solución de Problemas

### Error: "No se pudo abrir el puerto serie"
- Verificar que el STM32 está conectado: `ls /dev/ttyACM* /dev/ttyUSB*`
- Verificar permisos: `groups` (debe incluir `dialout`)
- Cerrar otros programas que usen el puerto (STM32CubeIDE, minicom, etc.)

### Error: "No module named 'serial'"
```bash
pip3 install pyserial
```

### El brazo no se mueve al enviar comandos
1. Verificar que el STM32 recibe los comandos: conectar un monitor serie y
   verificar que llegan las líneas `CMD:...`
2. Verificar que el firmware del STM32 está cargado correctamente
3. Verificar la calibración de encoders y potenciómetros

### La simulación no refleja el hardware
- Verificar que `sync_to_sim:=true` está activo en el launch
- Verificar que el STM32 envía líneas de feedback con formato `FB:...`
- Monitorear: `ros2 topic echo /hardware/joint_states`

## 📝 Notas

- El baudrate recomendado con STM32 es 115200 bps (vs 9600 del Arduino legado)
- Calibrar `pot_min`, `pot_max` y `*_ticks_per_rev` con el hardware real antes
  del primer movimiento
- El firmware STM32 debe implementar PID de posición; el control de bajo nivel
  (velocidad de motor) queda en el microcontrolador

## 📄 Licencia

[Agrega tu licencia aquí]

## 👤 Autor

Said Torres Cervantes

