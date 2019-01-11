#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Empty
from tf2_msgs.msg import TFMessage

import numpy as np
import transformations as xf

import kf
import util

# Transformation notation:
#   t_destination_source is a transform
#   vector_destination = t_destination_source * vector_source
#   xxx_f_destination means xxx is expressed in destination frame
#   xxx_pose_f_destination is equivalent to t_destination_xxx
#   t_a_c = t_a_b * t_b_c


def pose_to_matrix(p: Pose) -> np.ndarray:
    """geometry_msgs.msg.Pose to 4x4 matrix"""
    t_matrix = xf.translation_matrix([p.position.x, p.position.y, p.position.z])
    r_matrix = xf.quaternion_matrix([p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z])
    return np.dot(t_matrix, r_matrix)


def matrix_to_pose(m: np.ndarray) -> Pose:
    """4x4 matrix to geometry_msgs.msg.Pose"""
    t = xf.translation_from_matrix(m)
    q = xf.quaternion_from_matrix(m)  # Order is [w, x, y, z]
    return Pose(position=Point(x=t[0], y=t[1], z=t[2]), orientation=Quaternion(x=q[1], y=q[2], z=q[3], w=q[0]))


class Filter(Node):
    """
    Estimate the current pose and velocity using a Kalman filter.

    Subscriptions:
        start_mission (std_msgs/Empty)
        stop_mission (std_msgs/Empty)
        camera_pose (geometry_msgs/PoseWithCovarianceStamped)

    Publications:
        filtered_odom (nav_msgs/Odometry)
        /tf (tf2_msgs/TFMessage)
        estimated_path (nav_msgs/Path)
    """

    STATE_DIM = 18
    MEASUREMENT_DIM = 6

    FILTERED_ODOM_TOPIC = 'filtered_odom'
    ESTIMATED_PATH_TOPIC = 'estimated_path'
    TF_TOPIC = '/tf'

    def __init__(self):
        super().__init__('filter')

        self._last_msg_time = None
        self._mission_active = False

        self._filter = kf.KalmanFilter(state_dim=self.STATE_DIM, measurement_dim=self.MEASUREMENT_DIM)

        self._filter.H = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        ])

        self._filter.Q = np.eye(self.STATE_DIM) * 0.0001  # TODO revisit

        self.create_subscription(Empty, 'start_mission', self._start_mission_callback)
        self.create_subscription(Empty, 'stop_mission', self._stop_mission_callback)
        self.create_subscription(PoseWithCovarianceStamped, 'camera_pose', self._camera_pose_callback)

        self._filtered_odom_pub = self.create_publisher(Odometry, self.FILTERED_ODOM_TOPIC)
        self._path_pub = self.create_publisher(Path, self.ESTIMATED_PATH_TOPIC)
        self._tf_pub = self.create_publisher(TFMessage, self.TF_TOPIC)

        # Allocate messages
        self._filtered_odom_msg = Odometry()
        self._tf_msg = TFMessage()
        self._tf_msg.transforms.append(TransformStamped())
        self._path_msg = Path()
        self._path_msg.header.frame_id = 'map'

        # Tbc transform
        self._t_camera_base = xf.inverse_matrix(pose_to_matrix(Pose(
            position=Point(x=0.035, y=0., z=0.),
            orientation=Quaternion(x=-0.5, y=0.5, z=-0.5, w=0.5))))

        self.get_logger().info('filter.py ready')

    def _start_mission_callback(self, _msg: Empty):
        self._path_msg.poses.clear()
        self._mission_active = True

    def _stop_mission_callback(self, _msg: Empty):
        self._mission_active = False

    def _camera_pose_callback(self, msg: PoseWithCovarianceStamped):
        if not self._last_msg_time:
            self._last_msg_time = msg.header.stamp
            return

        dt = util.duration(self._last_msg_time, msg.header.stamp)
        if dt < 0:
            self.get_logger().error('time going backwards? dt = {}'.format(dt))

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

        # Get drone pose
        t_map_camera = pose_to_matrix(msg.pose.pose)
        t_map_base = t_map_camera.dot(self._t_camera_base)
        drone_pose_f_map = matrix_to_pose(t_map_base)

        p = drone_pose_f_map.position
        q = drone_pose_f_map.orientation
        e = xf.euler_from_quaternion([q.w, q.x, q.y, q.z])
        z = np.array([[p.x, p.y, p.z, e[0], e[1], e[2]]]).T
        R = np.array(msg.pose.covariance).reshape((self.MEASUREMENT_DIM, self.MEASUREMENT_DIM))

        x, P = self._filter.update(z, R)

        self._filtered_odom_msg.header = msg.header
        self._filtered_odom_msg.child_frame_id = 'base_link'
        self._filtered_odom_msg.pose.pose.position.x = x[0, 0]
        self._filtered_odom_msg.pose.pose.position.y = x[1, 0]
        self._filtered_odom_msg.pose.pose.position.z = x[2, 0]
        q = xf.quaternion_from_euler(ai=x[3, 0], aj=x[4, 0], ak=x[5, 0])
        self._filtered_odom_msg.pose.pose.orientation.x = q[1]
        self._filtered_odom_msg.pose.pose.orientation.y = q[2]
        self._filtered_odom_msg.pose.pose.orientation.z = q[3]
        self._filtered_odom_msg.pose.pose.orientation.w = q[0]
        self._filtered_odom_msg.pose.covariance = P[0:6, 0:6].flatten().tolist()
        self._filtered_odom_msg.twist.twist.linear.x = x[6, 0]
        self._filtered_odom_msg.twist.twist.linear.y = x[7, 0]
        self._filtered_odom_msg.twist.twist.linear.z = x[8, 0]
        self._filtered_odom_msg.twist.twist.angular.x = x[9, 0]
        self._filtered_odom_msg.twist.twist.angular.y = x[10, 0]
        self._filtered_odom_msg.twist.twist.angular.z = x[11, 0]
        self._filtered_odom_msg.twist.covariance = P[6:12, 6:12].flatten().tolist()

        # Publish filtered_odom with both pose and twist
        if self.count_subscribers(self.FILTERED_ODOM_TOPIC) > 0:
            self._filtered_odom_pub.publish(self._filtered_odom_msg)

        # Publish path
        if self._mission_active and self.count_subscribers(self.ESTIMATED_PATH_TOPIC) > 0:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'base_link'
            pose_stamped.header.stamp = msg.header.stamp
            util.copy_pose_to_pose(self._filtered_odom_msg.pose.pose, pose_stamped.pose)
            self._path_msg.poses.append(pose_stamped)
            self._path_msg.header.stamp = msg.header.stamp
            self._path_pub.publish(self._path_msg)

        # Publish tf
        if self.count_subscribers(self.TF_TOPIC) > 0:
            self._tf_msg.transforms[0].header = msg.header
            self._tf_msg.transforms[0].child_frame_id = 'base_link'
            util.copy_pose_to_transform(self._filtered_odom_msg.pose.pose, self._tf_msg.transforms[0].transform)
            self._tf_pub.publish(self._tf_msg)


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
