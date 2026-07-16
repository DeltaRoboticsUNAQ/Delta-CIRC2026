# src/arm_tasks/arm_tasks/rover_cooked_task.py
from arm_tasks.base_task import BaseTask
from rclpy.node import Node

class RoverCookedTask(BaseTask):
    def __init__(self, node: Node):
        super().__init__(node, "RoverCooked")

    def execute(self) -> bool:
        self.log_info("RoverCooked Task — pendiente de implementación")
        return self.arm.go_home()