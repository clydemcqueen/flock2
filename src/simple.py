#!/usr/bin/env python

import rclpy
from rclpy.node import Node

# The simplest ROS2 node using classes and methods

# How to add a new Python node:
#
# 1. create node file, e.g., flock2/src/new_node.py
#    file must be executable: chmod +x flock2/src/new_node.py
#    first line: #!/usr/bin/env python
#
# 2. add src/new_node.py to install section of CMakeLists.txt
#
# 3. add node to various launch descriptions, e.g., teleop_launch.py
#
# 4. build: colcon build --symlink-install --event-handlers console_direct+


class Simple(Node):

    def __init__(self):
        super().__init__('simple')


def main(args=None):
    rclpy.init(args=args)
    node = Simple()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C detected, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
