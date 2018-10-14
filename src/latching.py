#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String


class Latching(Node):

    def __init__(self):
        super().__init__('latching')
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        pub = self.create_publisher(String, 'foo', qos_profile=latching_qos)
        msg = String(data='test')
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Latching()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C detected, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
