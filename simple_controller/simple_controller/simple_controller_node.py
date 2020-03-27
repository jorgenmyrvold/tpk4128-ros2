import numpy as np
import time
from copy import copy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_interfaces.msg import PositionCommand


class SimpleControllerNode(Node):
    def __init__(self):
        super().__init__('simple_robot_controller')
        self._pub = self.create_publisher(PositionCommand, 'command', 10)
        self._sub = self.create_subscription(
            JointState, 'joint_states', self._callback, 10)

        self._t0 = None
        self._initial_position = None

    def _callback(self, msg):
        if self._t0 is None:
            self._t0 = time.time()
            self._initial_position = copy(msg.position)

        command_msg = PositionCommand()
        command_msg.command = copy(msg.position)

        correction = 0.1 * (np.sin(3.0 * (time.time() - self._t0)))
        command_msg.command[0] = self._initial_position[0] + correction

        self._pub.publish(command_msg)


def main(args=None):
    rclpy.init(args=args)

    simple_controller_node = SimpleControllerNode()
    rclpy.spin(simple_controller_node)
    
    simple_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()