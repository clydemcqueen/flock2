#!/usr/bin/env python

import time

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, Vector3, TransformStamped
from tf2_msgs.msg import TFMessage

# Testing a tf2 publisher


def now():
    cpu_time = time.time()
    sec = int(cpu_time)
    nanosec = int((cpu_time - sec) * 1000000)
    return Time(sec=sec, nanosec=nanosec)


class TF2Test(Node):

    def __init__(self):
        super().__init__('tf2_test')
        self._tf_pub = self.create_publisher(TFMessage, '/tf')

    def publish_tf(self, x):
        geometry_msg = TransformStamped()
        geometry_msg.header.frame_id = 'odom'
        geometry_msg.header.stamp = now()
        geometry_msg.child_frame_id = 'base_link'
        geometry_msg.transform.rotation = Quaternion(x=0., y=0., z=0., w=1.)
        geometry_msg.transform.translation = Vector3(x=float(x), y=0., z=0.)
        tf2_msg = TFMessage()
        tf2_msg.transforms.append(geometry_msg)
        self._tf_pub.publish(tf2_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TF2Test()
    log = node.get_logger()
    for x in range(100):
        log.info('publishing tf %d' % x)
        node.publish_tf(x / 100.)
        rclpy.spin_once(node, timeout_sec=1)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
