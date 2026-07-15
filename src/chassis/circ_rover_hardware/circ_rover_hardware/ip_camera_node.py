#!/usr/bin/env python3
"""
ip_camera_node.py  ──  paquete sugerido: circ_rover_hardware  (corre en la JETSON)

Abre UNA cámara IP (RTSP o MJPEG; OpenCV maneja ambos) y la republica como
sensor_msgs/Image en ROS 2. Luego web_video_server la sirve como MJPEG hacia la
UI remota. Se corre UNA instancia por cámara (ver cameras.launch.py).

¿Por qué este nodo y no MJPEG directo al navegador?
  Operas REMOTO: tu laptop no alcanza las IPs internas del rover. Todo el video
  pasa por la Jetson -> web_video_server -> un solo puerto hacia tu laptop.

Parámetros:
  url           URL completa de la cámara. Ejemplos:
                  rtsp://user:pass@192.168.1.50:554/Streaming/Channels/101
                  http://user:pass@192.168.1.50/mjpg/video.mjpg
  topic         tópico de salida (default 'image_raw' bajo el namespace del nodo)
  frame_id      frame del header (default = nombre del nodo)
  fps           tope de publicación en Hz (default 15; baja para ahorrar ancho de banda)
  resize_width  si >0, reescala a ese ancho (reduce ancho de banda; default 0 = sin reescalar)
  use_gstreamer si True, usa pipeline GStreamer con decode por HW (nvv4l2decoder).
                Requiere OpenCV con soporte GStreamer (el de JetPack lo trae) y que
                la cámara sea RTSP/h264.

Ejemplo suelto:
  ros2 run circ_rover_hardware ip_camera_node --ros-args \
      -r __node:=ipcam_1 \
      -p url:='rtsp://admin:1234@192.168.1.51:554/Streaming/Channels/101' \
      -p topic:=image_raw -p fps:=15
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2

try:
    from cv_bridge import CvBridge
    HAVE_BRIDGE = True
except Exception:
    HAVE_BRIDGE = False


def gst_h264_pipeline(url, latency=100):
    """Pipeline GStreamer con decodificación H264 por hardware (Jetson)."""
    return (
        f'rtspsrc location={url} latency={latency} ! '
        'rtph264depay ! h264parse ! nvv4l2decoder ! '
        'nvvidconv ! video/x-raw,format=BGRx ! '
        'videoconvert ! video/x-raw,format=BGR ! '
        'appsink drop=true sync=false max-buffers=1'
    )


class IpCameraNode(Node):
    def __init__(self):
        super().__init__('ip_camera')

        self.declare_parameter('url', '')
        self.declare_parameter('topic', 'image_raw')
        self.declare_parameter('frame_id', '')
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('resize_width', 0)
        self.declare_parameter('use_gstreamer', False)

        self.url = self.get_parameter('url').value
        topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value or self.get_name()
        self.fps = float(self.get_parameter('fps').value)
        self.resize_w = int(self.get_parameter('resize_width').value)
        self.use_gst = bool(self.get_parameter('use_gstreamer').value)

        if not self.url:
            self.get_logger().error('Parámetro "url" vacío. Nada que abrir.')
        if not HAVE_BRIDGE:
            self.get_logger().error('cv_bridge no disponible: sudo apt install ros-humble-cv-bridge')

        self.bridge = CvBridge() if HAVE_BRIDGE else None
        self.pub = self.create_publisher(Image, topic, 10)
        self.cap = None
        self._open()

        period = 1.0 / max(self.fps, 1.0)
        self.timer = self.create_timer(period, self.tick)
        self.get_logger().info(f'ip_camera_node [{self.get_name()}] -> {topic} @ {self.fps:.0f} Hz')

    def _open(self):
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        if self.use_gst:
            self.cap = cv2.VideoCapture(gst_h264_pipeline(self.url), cv2.CAP_GSTREAMER)
        else:
            # FFMPEG backend: sirve para RTSP h264 y para MJPEG/HTTP.
            self.cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
            # Buffer chico = menos latencia (si el backend lo respeta)
            try:
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass
        if self.cap and self.cap.isOpened():
            self.get_logger().info(f'[{self.get_name()}] stream abierto')
        else:
            self.get_logger().warn(f'[{self.get_name()}] no se pudo abrir; reintentando…')

    def tick(self):
        if self.cap is None or not self.cap.isOpened():
            self._open()
            return
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn(f'[{self.get_name()}] frame perdido; reabriendo stream')
            self._open()
            return
        if self.resize_w > 0 and frame.shape[1] > self.resize_w:
            h = int(frame.shape[0] * self.resize_w / frame.shape[1])
            frame = cv2.resize(frame, (self.resize_w, h))
        if self.bridge is None:
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IpCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()