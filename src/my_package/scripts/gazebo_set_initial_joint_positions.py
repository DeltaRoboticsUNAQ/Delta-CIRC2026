#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from gazebo_msgs.srv import SetModelConfiguration


class GazeboSetInitialJointPositions(Node):
    def __init__(self) -> None:
        super().__init__('gazebo_set_initial_joint_positions')

        self.declare_parameter('service_name', '/gazebo/set_model_configuration')
        self.declare_parameter('model_name', 'my_arm')
        self.declare_parameter('urdf_param_name', 'robot_description')

        # Joint names must match the URDF
        self.declare_parameter('joint_names', [
            'bracket_joint',
            'humerus_low_joint',
            'forearm_low_joint',
            'ubracket_joint',
            'endeffector_joint',
        ])
        # Joint positions (radians)
        self.declare_parameter('joint_positions', [0.0, 0.0, 0.0, 0.0, 0.0])

        service_name = self.get_parameter('service_name').get_parameter_value().string_value
        self._client = self.create_client(SetModelConfiguration, service_name)

        self.get_logger().info(f'Waiting for service: {service_name}')
        if not self._client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'Service not available: {service_name}')

        req = SetModelConfiguration.Request()
        req.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        req.urdf_param_name = self.get_parameter('urdf_param_name').get_parameter_value().string_value
        req.joint_names = list(self.get_parameter('joint_names').get_parameter_value().string_array_value)
        req.joint_positions = list(self.get_parameter('joint_positions').get_parameter_value().double_array_value)

        if len(req.joint_names) != len(req.joint_positions):
            raise ValueError('joint_names and joint_positions must have same length')

        self.get_logger().info(
            'Setting initial joint positions for %s: %s'
            % (req.model_name, dict(zip(req.joint_names, req.joint_positions)))
        )

        future = self._client.call_async(req)
        future.add_done_callback(self._done)

    def _done(self, future) -> None:
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f'Success: {resp.status_message}')
            else:
                self.get_logger().error(f'Failed: {resp.status_message}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Exception calling service: {exc}')
        finally:
            rclpy.shutdown()


def main() -> None:
    rclpy.init()
    try:
        node = GazeboSetInitialJointPositions()
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
