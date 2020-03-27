import numpy as np
import time
from copy import copy
import cv2 as cv

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class OpenCVCameraNode(Node):
    def __init__(self):
        super().__init__('opencv_camera')
        self._pub = self.create_publisher(Image, 'image', 10)
        self._camera = cv.VideoCapture(0)

    def _callback(self):
        img, ret = self._camera.read()
        self._pub(img)
    
    def __del__(self):
        self._camera.release()


def main(args=None):
    rclpy.init(args=args)

    camera_node = OpenCVCameraNode()
    rclpy.spin(camera_node)
    
    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()