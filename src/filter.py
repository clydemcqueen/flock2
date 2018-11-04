#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Empty
from tf2_msgs.msg import TFMessage

import numpy as np
import transformations as xf

import kf
import util


class Filter(Node):
    """
    Apply a Kalman filter to odometry messages.
    18 state variables: 6 pose (measured), 6 velocity (hidden) and 6 acceleration (hidden).
    Subscribe to 'odom' and publish 'filtered_odom'. Publish computed velocity.
    Also publish 'odom' to 'base_link transform.
    """

    def __init__(self):
        super().__init__('filter')

        self._last_msg_time = None
        self._filter = kf.KalmanFilter(state_dim=18, measurement_dim=6)

        self._filter.H = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        ])

        self._filter.Q = np.eye(18) * 0.0001  # TODO revisit

        best_effort = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)

        self.create_subscription(Empty, 'start_mission', self._start_mission_callback)
        self.create_subscription(Empty, 'stop_mission', self._stop_mission_callback)
        self.create_subscription(Odometry, 'odom', self._odom_callback, qos_profile=best_effort)

        self._filtered_odom_pub = self.create_publisher(Odometry, 'filtered_odom', qos_profile=best_effort)
        self._tf_pub = self.create_publisher(TFMessage, '/tf')

        # Publish a path if we're in a mission
        self._path_pub = self.create_publisher(Path, 'estimated_path')
        self._publish_path = False
        self._path = Path()
        self._path.header.frame_id = 'odom'

    def _start_mission_callback(self, msg: Empty):
        self._path.poses.clear()
        self._publish_path = True

    def _stop_mission_callback(self, msg: Empty):
        self._publish_path = False

    def _odom_callback(self, msg: Odometry):
        if not self._last_msg_time:
            self._last_msg_time = msg.header.stamp
            return

        dt = util.duration(self._last_msg_time, msg.header.stamp)
        self._last_msg_time = msg.header.stamp
        dt2 = 0.5 * dt * dt

        self._filter.F = np.array([
            [1, 0, 0, 0, 0, 0,    dt, 0, 0, 0, 0, 0,    dt2, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0,    0, dt, 0, 0, 0, 0,    0, dt2, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0,    0, 0, dt, 0, 0, 0,    0, 0, dt2, 0, 0, 0],
            [0, 0, 0, 1, 0, 0,    0, 0, 0, dt, 0, 0,    0, 0, 0, dt2, 0, 0],
            [0, 0, 0, 0, 1, 0,    0, 0, 0, 0, dt, 0,    0, 0, 0, 0, dt2, 0],
            [0, 0, 0, 0, 0, 1,    0, 0, 0, 0, 0, dt,    0, 0, 0, 0, 0, dt2],

            [0, 0, 0, 0, 0, 0,    1, 0, 0, 0, 0, 0,     dt, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0,    0, 1, 0, 0, 0, 0,     0, dt, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0,    0, 0, 1, 0, 0, 0,     0, 0, dt, 0, 0, 0],
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 1, 0, 0,     0, 0, 0, dt, 0, 0],
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 1, 0,     0, 0, 0, 0, dt, 0],
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 1,     0, 0, 0, 0, 0, dt],

            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 1],
        ])

        self._filter.predict()

        # Ignore incoming twist
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        e = xf.euler_from_quaternion([q.w, q.x, q.y, q.z])
        z = np.array([[p.x, p.y, p.z, e[0], e[1], e[2]]]).T
        R = np.array(msg.pose.covariance).reshape((6, 6))

        x, P = self._filter.update(z, R)

        filtered_pose = Pose()
        filtered_pose.position.x = x[0, 0]
        filtered_pose.position.y = x[1, 0]
        filtered_pose.position.z = x[2, 0]
        q = xf.quaternion_from_euler(ai=x[3, 0], aj=x[4, 0], ak=x[5, 0])
        filtered_pose.orientation.x = q[1]
        filtered_pose.orientation.y = q[2]
        filtered_pose.orientation.z = q[3]
        filtered_pose.orientation.w = q[0]

        # Publish filtered_odom with both pose and twist
        filtered_odom_msg = Odometry()
        filtered_odom_msg.header = msg.header
        filtered_odom_msg.child_frame_id = msg.child_frame_id
        filtered_odom_msg.pose.pose = filtered_pose
        filtered_odom_msg.pose.covariance = P[0:6, 0:6].flatten().tolist()
        filtered_odom_msg.twist.twist.linear.x = x[6, 0]
        filtered_odom_msg.twist.twist.linear.y = x[7, 0]
        filtered_odom_msg.twist.twist.linear.z = x[8, 0]
        filtered_odom_msg.twist.twist.angular.x = x[9, 0]
        filtered_odom_msg.twist.twist.angular.y = x[10, 0]
        filtered_odom_msg.twist.twist.angular.z = x[11, 0]
        filtered_odom_msg.twist.covariance = P[6:12, 6:12].flatten().tolist()
        self._filtered_odom_pub.publish(filtered_odom_msg)

        # Publish tf
        tf_msg = TFMessage()
        tf_msg.transforms.append(util.pose_to_transform(filtered_pose, msg.header.stamp, 'odom', 'base_link'))
        self._tf_pub.publish(tf_msg)

        # Publish a path if we're in a mission
        if self._publish_path:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'base_link'
            pose_stamped.header.stamp = msg.header.stamp
            pose_stamped.pose = filtered_odom_msg.pose.pose
            self._path.header.stamp = msg.header.stamp
            self._path.poses.append(pose_stamped)
            self._path_pub.publish(self._path)


def main(args=None):
    rclpy.init(args=args)
    node = Filter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C detected, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
