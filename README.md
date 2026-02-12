# Robotic Arm ROS2 Workspace

Workspace de ROS2 para control y visualización de brazo robótico con Arduino.

## 📋 Requisitos

- ROS 2 Humble (Ubuntu 22.04)
- Python 3.10+
- PySerial
- Arduino con código de control de servos/actuadores lineales

## 🚀 Instalación

### En Ubuntu 22.04 / WSL2 / Máquina Virtual

1. **Instalar ROS 2 Humble** (si no está instalado):
```bash
# Agregar repositorio de ROS 2
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

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
sudo usermod -a -G dialout $USER  # Para acceso a puerto serial
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

### Control con Hardware (Arduino)
```bash
source install/setup.bash
ros2 run my_package arm_hw_teleop_bridge.py --ros-args -p port:=/dev/ttyACM0 -p baudrate:=9600
```

**Teclas de control:**
- `A/D` - Rotar base izquierda/derecha
- `W/S` - Muñeca arriba/abajo
- `Q/E` - Muñeca rotar izquierda/derecha
- `O/L` - Actuador 1 extender/retraer
- `I/K` - Actuador 2 extender/retraer
- `ESPACIO` - Detener todo

## 🖥️ Configuración para WSL2

Si usas WSL2, los dispositivos USB necesitan mapearse:

### Método 1: usbipd-win (Recomendado)

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

### Método 2: Puertos COM nativos

Los puertos COM de Windows se mapean así:
- COM1 → `/dev/ttyS0`
- COM2 → `/dev/ttyS1`
- COM7 → `/dev/ttyS6`
- etc.

Verifica tu puerto COM en Windows (Device Manager) y usa el equivalente `/dev/ttySX`.

## 🖥️ Configuración para Máquina Virtual

### VirtualBox / VMware

1. **Habilitar USB en la VM:**
   - VirtualBox: Settings → USB → Enable USB Controller (USB 2.0 o 3.0)
   - VMware: VM Settings → USB Controller → Enable USB compatibility

2. **Conectar el dispositivo USB a la VM:**
   - Devices → USB → [Tu Arduino]

3. **Verificar el dispositivo:**
```bash
ls /dev/ttyACM* /dev/ttyUSB*
dmesg | grep tty  # Ver logs de conexión
```

4. **Permisos:**
```bash
sudo usermod -a -G dialout $USER
# Cerrar sesión y volver a entrar
```

## 📦 Estructura del Proyecto

```
arm_ws/
├── src/
│   └── my_package/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── config/         # Configuraciones
│       ├── launch/         # Launch files
│       ├── meshes/         # Modelos 3D (si hay)
│       ├── scripts/        # Scripts Python
│       │   └── arm_hw_teleop_bridge.py
│       ├── src/            # Código C++ (si hay)
│       └── urdf/           # Modelos URDF del robot
├── build/                  # Archivos de compilación (ignorado)
├── install/                # Archivos instalados (ignorado)
└── log/                    # Logs de compilación (ignorado)
```

## 🐛 Solución de Problemas

### Error: "could not open port"
- Verifica que el puerto existe: `ls /dev/tty*`
- Verifica permisos: `groups` (debe incluir `dialout`)
- Cierra otros programas que usen el puerto (Arduino IDE, etc.)

### Error: "No module named 'serial'"
```bash
pip3 install pyserial
```

### En VM: Dispositivo USB no aparece
- Asegúrate de que las Guest Additions/Tools estén instaladas
- Habilita USB 2.0 o 3.0 en la configuración de la VM
- Reconecta el dispositivo USB físicamente

## 📝 Notas

- El código de Arduino debe estar cargado en el microcontrolador
- La velocidad de baudrate debe coincidir (9600 bps por defecto)
- En WSL2, la latencia puede ser mayor que en Linux nativo

## 📄 Licencia

[Agrega tu licencia aquí]

## 👤 Autor

Said Torres Cervantes
