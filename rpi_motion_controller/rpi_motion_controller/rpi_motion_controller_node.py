import socket
import numpy as np
import time
from copy import copy
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_interfaces.msg import PositionCommand

SCALING_FACTOR = 1
PI = 3.14

class RpiMotionControllerNode(Node):
    def __init__(self):
        super().__init__('rpi_motion_controller_node')

        self._sensor_host = '10.0.0.27'  # local IP-address for RBPI/server
        self._sensor_port = 50007        # Random socket, same as server
        self._sensor_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sensor_socket.connect((self._sensor_host, self._sensor_port))

        self._pub = self.create_publisher(PositionCommand, 'command', 10)
        self._sub = self.create_subscription(
            JointState, 'joint_states', self._callback, 10)
        
    def __del__(self):
        self._sensor_socket.close()

    def _callback(self, msg):
        command_msg = PositionCommand()
        command_msg.command = copy(msg.position)

        event = self._sensor_socket.recv(32)
        controller_possition = list(map(float, event.strip().split(b" ")))

        #command_msg.command[0] = PI * controller_possition[0]               # Base rotation joint
        command_msg.command[1] = PI + (PI/2)*(1 + controller_possition[0])  # Arm arm joint
        command_msg.command[2] = (PI/2)*(controller_possition[1])           # Midle arm joint
        #command_msg.command[3] = PI * (controller_possition[0])             # Midle rotation joint
        command_msg.command[4] = (PI/2)*(controller_possition[2])           # Outer arm joint 
        #command_msg.command[5] = PI * (controller_possition[0])             # Outer rotation joint

        # Uncomment to reset arm to uppright
        command_msg.command[0] = 0
        #command_msg.command[1] = -PI/2
        #command_msg.command[2] = 0
        command_msg.command[3] = 0 
        #command_msg.command[4] = 0
        command_msg.command[5] = 0

        self._pub.publish(command_msg)
        print("command_msg: ", command_msg.command)


def main(args=None):
    rclpy.init(args=args)

    rpi_motion_contoller_node = RpiMotionControllerNode()
    rclpy.spin(rpi_motion_contoller_node)

    rpi_motion_contoller_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
