#!/usr/bin/env python3

import sys
import math

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException

class TfListener(Node):

    def __init__(self):
        super().__init__('tf_listener')
        self.target = "tool0"
        self.source = "base_link"
        self.get_logger().info("Transforming from {} to {}".format(self.source, self.target))
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.publisher_ = self.create_publisher(TransformStamped, "/tool_pose",10)
        self.timer = self.create_timer(0.1, self.timer_callback) #30 Hz = 0.333s

    def timer_callback(self):
        try:
            trans = self._tf_buffer.lookup_transform(self.source, self.target, rclpy.time.Time())
            self.publisher_.publish(trans)

        except LookupException as e:
            self.get_logger().error('failed to get transform {} \n'.format(repr(e)))

def main(argv=sys.argv):
    rclpy.init(args=argv)
    node = TfListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
