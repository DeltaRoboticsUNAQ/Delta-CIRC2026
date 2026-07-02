#!/usr/bin/env python3
"""
cameras.launch.py  ──  paquete sugerido: circ_rover_hardware  (corre en la JETSON)

Levanta las fuentes de cámara que NO son la OAK:
  • 4 cámaras IP  -> ip_camera_node (una por cámara) -> /ipcam_N/image_raw
  • 1 cámara USB  -> v4l2_camera                      -> /usb_cam/image_raw

La OAK (RGB + /scan del LiDAR) la levantas con tu launch de depthai por separado.
web_video_server y rosbridge ya van en ui_bringup.launch.py.

  *** EDITA las 4 URLs de abajo con las de tus cámaras ***

Cómo descubrir la URL si no la sabes:
  1) Entra al panel web de la cámara (su IP en el navegador, dentro de la red del rover).
  2) O usa ONVIF Device Manager (Windows) / `onvif-cli` para listar el stream.
  3) Patrones RTSP comunes por marca:
       Hikvision : rtsp://user:pass@IP:554/Streaming/Channels/101
       Dahua     : rtsp://user:pass@IP:554/cam/realmonitor?channel=1&subtype=0
       Axis      : rtsp://user:pass@IP/axis-media/media.amp
       Genérica  : rtsp://user:pass@IP:554/stream1
  4) Verifica desde la Jetson antes de meterla a ROS:
       ffprobe "rtsp://user:pass@IP:554/..."     # te dice códec (h264/mjpeg) y resolución

Instalación:
    sudo apt install -y ros-humble-v4l2-camera ros-humble-cv-bridge
    # ffmpeg suele venir; si no:  sudo apt install -y ffmpeg

Uso:
    ros2 launch circ_rover_hardware cameras.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node

# ── EDITA AQUÍ ────────────────────────────────────────────────────────────────
IP_CAMS = [
    {'node': 'ipcam_1', 'url': 'rtsp://admin:pass@192.168.1.51:554/Streaming/Channels/101'},
    {'node': 'ipcam_2', 'url': 'rtsp://admin:pass@192.168.1.52:554/Streaming/Channels/101'},
    {'node': 'ipcam_3', 'url': 'rtsp://admin:pass@192.168.1.53:554/Streaming/Channels/101'},
    {'node': 'ipcam_4', 'url': 'rtsp://admin:pass@192.168.1.54:554/Streaming/Channels/101'},
]
# Si confirmas que son RTSP/h264, pon True para usar el decodificador por HW de la Jetson:
USE_GSTREAMER = False
# Tope de FPS y ancho de reescalado (bajan el ancho de banda hacia tu laptop remota):
FPS = 15.0
RESIZE_WIDTH = 640      # 0 = sin reescalar
USB_DEVICE = '/dev/video0'
# ──────────────────────────────────────────────────────────────────────────────


def generate_launch_description():
    nodes = []

    for cam in IP_CAMS:
        nodes.append(Node(
            package='circ_rover_hardware',
            executable='ip_camera_node',
            name=cam['node'],
            output='screen',
            parameters=[{
                'url': cam['url'],
                'topic': 'image_raw',        # queda en /<node>/image_raw
                'frame_id': cam['node'],
                'fps': FPS,
                'resize_width': RESIZE_WIDTH,
                'use_gstreamer': USE_GSTREAMER,
            }],
        ))

    # Cámara USB
    nodes.append(Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': USB_DEVICE,
            'image_size': [640, 480],
            'camera_frame_id': 'usb_cam',
        }],
        remappings=[('/image_raw', '/usb_cam/image_raw')],
    ))

    return LaunchDescription(nodes)