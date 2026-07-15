# src/arm_tasks/arm_tasks/heist_task.py
from arm_tasks.base_task import BaseTask
from rclpy.node import Node

class HeistTask(BaseTask):
    def __init__(self, node: Node):
        super().__init__(node, "Heist")

    def execute(self) -> bool:
        self.log_info("Heist Task — pendiente de implementación")
        return self.arm.go_home()