#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self) -> None:
        super().__init__('robot_description_publisher')

        self.declare_parameter('robot_description', '')
        self.declare_parameter('topic', 'robot_description')
        self.declare_parameter('publish_period_s', 1.0)

        topic = self.get_parameter('topic').get_parameter_value().string_value
        publish_period_s = self.get_parameter('publish_period_s').get_parameter_value().double_value

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(String, topic, qos)

        self._timer = self.create_timer(publish_period_s, self._publish_once)
        self.get_logger().info(f'Publishing robot_description on topic: {topic}')

    def _publish_once(self) -> None:
        robot_description = self.get_parameter('robot_description').get_parameter_value().string_value
        if not robot_description:
            self.get_logger().warn('robot_description parameter is empty')
            return

        msg = String()
        msg.data = robot_description
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = RobotDescriptionPublisher()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
