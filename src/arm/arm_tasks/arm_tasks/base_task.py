# src/arm_tasks/arm_tasks/base_task.py
"""
Clase base para todas las tasks de competencia CIRC 2026.
Define la interfaz común y utilidades compartidas.
"""

from abc import ABC, abstractmethod
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from arm_tasks.arm_motion_client import ArmMotionClient


class BaseTask(ABC):
    """
    Clase abstracta base para todas las tasks de competencia.
    Provee: arm client, logging, estado, publicación de status.
    """

    def __init__(self, node: Node, task_name: str):
        self.node = node
        self.task_name = task_name
        self.arm = ArmMotionClient(node)
        self.success = False
        self._start_time = None

        # Publisher de status para la UI del operador
        self._status_pub = node.create_publisher(
            String, '/competition/task_status', 10
        )

        self.log_info(f"Task '{task_name}' inicializada")

    @abstractmethod
    def execute(self) -> bool:
        """
        Ejecuta la task completa.
        Retorna True si completó exitosamente.
        """
        pass

    def log_info(self, msg: str):
        self.node.get_logger().info(f"[{self.task_name}] {msg}")
        self._publish_status(f"INFO: {msg}")

    def log_warn(self, msg: str):
        self.node.get_logger().warn(f"[{self.task_name}] {msg}")
        self._publish_status(f"WARN: {msg}")

    def log_error(self, msg: str):
        self.node.get_logger().error(f"[{self.task_name}] {msg}")
        self._publish_status(f"ERROR: {msg}")

    def wait(self, seconds: float, reason: str = ""):
        """Espera no bloqueante con logging."""
        if reason:
            self.log_info(f"Esperando {seconds}s — {reason}")
        time.sleep(seconds)

    def elapsed_time(self) -> float:
        """Tiempo transcurrido desde inicio de la task."""
        if self._start_time is None:
            return 0.0
        return time.time() - self._start_time

    def _start_timer(self):
        self._start_time = time.time()

    def _publish_status(self, msg: str):
        status = String()
        status.data = f"[{self.task_name}] {msg}"
        self._status_pub.publish(status)

    def go_home_safe(self) -> bool:
        """
        Vuelve a HOME siempre, incluso si hubo error.
        Usar en finally blocks.
        """
        self.log_info("Volviendo a HOME (seguridad)")
        return self.arm.go_home(duration_sec=3.0)