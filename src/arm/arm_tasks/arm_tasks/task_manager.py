# src/arm_tasks/arm_tasks/task_manager.py
"""
Task Manager central para competencia CIRC 2026.
Recibe comandos del operador y ejecuta las tasks correspondientes.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import threading


class TaskManager(Node):

    def __init__(self):
        super().__init__('task_manager')

        self._current_task = None
        self._task_thread = None
        self._running = False

        # Suscripción para activar tasks desde operador
        self.create_subscription(
            String,
            '/competition/set_task',
            self._task_command_cb,
            10
        )

        # Publisher de estado
        self._status_pub = self.create_publisher(
            String, '/competition/task_status', 10
        )

        # Servicios de control de emergencia
        self.create_service(
            Trigger, '/arm/stop_task', self._stop_task_cb
        )
        self.create_service(
            Trigger, '/arm/go_home', self._go_home_cb
        )

        self.get_logger().info(
            "╔══════════════════════════════╗\n"
            "║  Task Manager CIRC 2026 listo ║\n"
            "║  Esperando comandos...        ║\n"
            "╚══════════════════════════════╝\n"
            "Comandos disponibles:\n"
            "  ros2 topic pub /competition/set_task std_msgs/String "
            "\"data: 'snack_run'\" --once\n"
            "  ros2 topic pub /competition/set_task std_msgs/String "
            "\"data: 'rover_cooked'\" --once\n"
            "  ros2 topic pub /competition/set_task std_msgs/String "
            "\"data: 'heist'\" --once\n"
            "  ros2 topic pub /competition/set_task std_msgs/String "
            "\"data: 'home'\" --once"
        )

    def _task_command_cb(self, msg: String):
        """Recibe comando de task del operador."""
        cmd = msg.data.lower().strip()
        self.get_logger().info(f"Comando recibido: '{cmd}'")

        # Si hay una task corriendo, no iniciar otra
        if self._task_thread and self._task_thread.is_alive():
            self.get_logger().warn(
                f"Task en ejecución. "
                f"Envía '/arm/stop_task' para detenerla."
            )
            return

        # Ejecutar en hilo separado para no bloquear ROS
        self._task_thread = threading.Thread(
            target=self._run_task,
            args=(cmd,),
            daemon=True
        )
        self._task_thread.start()

    def _run_task(self, task_name: str):
        """Ejecuta la task en hilo separado."""
        self._running = True
        success = False

        try:
            if task_name == 'snack_run':
                from arm_tasks.snack_run_task import SnackRunTask
                task = SnackRunTask(self)
                success = task.execute()

            elif task_name == 'rover_cooked':
                from arm_tasks.rover_cooked_task import RoverCookedTask
                task = RoverCookedTask(self)
                success = task.execute()

            elif task_name == 'heist':
                from arm_tasks.heist_task import HeistTask
                task = HeistTask(self)
                success = task.execute()

            elif task_name == 'home':
                # Comando rápido para ir a HOME
                from arm_tasks.arm_motion_client import ArmMotionClient
                arm = ArmMotionClient(self)
                success = arm.go_home(duration_sec=3.0)

            else:
                self.get_logger().warn(
                    f"Task desconocida: '{task_name}'\n"
                    f"Disponibles: snack_run, rover_cooked, heist, home"
                )
                return

            status = "✅ COMPLETADA" if success else "❌ FALLIDA"
            self.get_logger().info(f"Task '{task_name}': {status}")

        except Exception as e:
            self.get_logger().error(
                f"Excepción en task '{task_name}': {e}"
            )
            import traceback
            self.get_logger().error(traceback.format_exc())

        finally:
            self._running = False

    def _stop_task_cb(self, request, response):
        """Detiene la task actual (lo más seguro posible)."""
        self._running = False
        response.success = True
        response.message = "Señal de stop enviada"
        self.get_logger().warn("⚠️ STOP solicitado por operador")
        return response

    def _go_home_cb(self, request, response):
        """Mueve el brazo a HOME inmediatamente."""
        from arm_tasks.arm_motion_client import ArmMotionClient
        arm = ArmMotionClient(self)
        ok = arm.go_home(duration_sec=3.0)
        response.success = ok
        response.message = "HOME ejecutado" if ok else "Error en HOME"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()