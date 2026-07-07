#!/usr/bin/env python3
"""
ui_bringup.launch.py  ──  paquete sugerido: circ_rover_ui (o donde te acomode)

Levanta SOLO el puente entre tu grafo de ROS 2 y el rover_ui_ros2.html:

    rosbridge_server  (ws://<host>:9090)  <-- roslibjs del HTML habla aquí
      └─ rosapi           (para el panel "Nodos" via /rosapi/nodes)
    web_video_server  (http://<host>:8080) <-- feeds MJPEG de cámara

NO arranca tu localización ni tu base; eso lo sigues lanzando con tu bringup
normal (EKF, imu_bridge, wheel_bridge, skid_steer_base, gps, etc.). Este launch
es complementario: córrelo EN PARALELO a tu stack ya operativo.

Instalación de dependencias (una vez, en la Jetson y/o WSL2):
    sudo apt update
    sudo apt install -y ros-humble-rosbridge-suite ros-humble-web-video-server

Uso:
    ros2 launch circ_rover_ui ui_bringup.launch.py
    # o directo, sin paquete:
    ros2 launch ./ui_bringup.launch.py

Luego abre el HTML en el navegador. Si lo abres en la MISMA máquina:
    file:///ruta/rover_ui_ros2.html        (toma ws://localhost:9090 solo)
Si lo abres desde OTRA laptop apuntando a la Jetson, pásale la IP por query:
    rover_ui_ros2.html?ros=ws://192.168.1.42:9090&video=http://192.168.1.42:8080
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rosbridge_port = LaunchConfiguration('rosbridge_port')
    video_port = LaunchConfiguration('video_port')

    return LaunchDescription([
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument('video_port', default_value='8080'),

        # ── rosbridge_server: WebSocket que habla protocolo rosbridge v2 ──
        # roslibjs (el del HTML) se conecta a ws://host:<rosbridge_port>.
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': rosbridge_port,
                # Sube si mandas imágenes grandes por el socket (no es el caso
                # si usas web_video_server para vídeo, que es lo recomendado).
                'max_message_size': 10_000_000,
                # Deja que rosbridge intente casar el QoS del publicador al
                # suscribirse (clave para tópicos de sensor BEST_EFFORT).
                'default_call_service_timeout': 5.0,
            }],
        ),

        # ── rosapi: expone /rosapi/nodes, /rosapi/topics, etc. ──
        # Lo usa el panel "Estado de nodos" del HTML (pollNodes()).
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
            output='screen',
        ),

        # ── web_video_server: convierte topics de imagen en streams MJPEG ──
        # El HTML pide  http://host:<video_port>/stream?topic=/oak/rgb/image_raw
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
            parameters=[{
                'port': video_port,
                'address': '0.0.0.0',
            }],
        ),
    ])