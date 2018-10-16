#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String


class LatchSub(Node):

    def __init__(self):
        super().__init__('latch_sub')
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.create_subscription(String, 'foo', self.callback, qos_profile=latching_qos)

    def callback(self, msg):
        self.get_logger().info('Heard %s' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = LatchSub()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C detected, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
