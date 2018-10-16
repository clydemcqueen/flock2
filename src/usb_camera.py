#!/usr/bin/env python

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Transform, TransformStamped, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

import cv2
import numpy

import detect_aruco

# Grab frames from a USB camera and publish them as ROS messages at 1Hz for testing


def now():
    cpu_time = time.time()
    sec = int(cpu_time)
    nanosec = int((cpu_time - sec) * 1000000)
    return Time(sec=sec, nanosec=nanosec)


class CameraTest(Node):

    def __init__(self):
        super().__init__('camera_test')

        # Clyde's USB camera
        camera_matrix = numpy.array(
            [[460.465192, 0.000000, 350.820241],
             [0.000000, 459.337700, 251.915294],
             [0.000000, 0.000000, 1.000000]])
        distortion = numpy.array([0.006521, -0.006982, 0.001500, 0.000761, 0.000000])

        # ArUco detector
        self._detector = detect_aruco.DetectArUco(camera_matrix, distortion)

        # Send some data "best effort"
        # Set "unreliable" in rviz2
        # Sadly these messages don't show up in `ros2 topic echo`
        sensor_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)

        # Publishers
        self._tf_pub = self.create_publisher(TFMessage, '/tf')
        self._rviz_markers_pub = self.create_publisher(MarkerArray, 'rviz_markers', qos_profile=sensor_qos)

        # Optionally publish images
        self._publish_images = False
        if self._publish_images:
            self._image_pub = self.create_publisher(Image, 'image_marked', qos_profile=sensor_qos)
            self._cv_bridge = CvBridge()
        else:
            self._image_pub = None
            self._cv_bridge = None

        self._video_thread = None
        self._stop_request = None

    def publish_tf(self, pose, stamp, child_frame):
        v = Vector3(x=pose.position.x, y=pose.position.y, z=pose.position.z)
        geometry_msg = TransformStamped()
        geometry_msg.header.frame_id = 'odom'
        geometry_msg.header.stamp = stamp
        geometry_msg.child_frame_id = child_frame
        geometry_msg.transform = Transform(translation=v, rotation=pose.orientation)
        tf2_msg = TFMessage()
        tf2_msg.transforms.append(geometry_msg)
        self._tf_pub.publish(tf2_msg)

    def start(self):
        self.get_logger().info("starting video thread")
        self._stop_request = threading.Event()
        self._video_thread = threading.Thread(target=self.video_worker)
        self._video_thread.start()

    def stop(self):
        self.get_logger().info("stopping video thread")
        self._stop_request.set()
        self._video_thread.join(timeout=2)
        self._stop_request = None
        self._video_thread = None

    def video_worker(self):
        cap = cv2.VideoCapture(0)

        while not self._stop_request.isSet():
            stamp = now()
            ret, frame = cap.read()
            if not ret:
                continue

            # Detect markers
            frame, drone_pose, marker_poses = self._detector.detect(self.get_logger(), frame)

            # Optionally publish images
            if self._publish_images:
                image_msg = self._cv_bridge.cv2_to_imgmsg(frame, 'bgr8')
                image_msg.header.stamp = stamp
                self._image_pub.publish(image_msg)
            else:
                cv2.imshow('usb_camera', frame)
                cv2.waitKey(1)

            # Publish transforms and rviz markers
            if drone_pose is not None and marker_poses is not None:
                self.publish_tf(drone_pose, stamp, child_frame='base_link')
                marker_array = MarkerArray()
                for marker_id, marker_pose in marker_poses.items():
                    marker = Marker()
                    marker.id = marker_id
                    marker.header.frame_id = 'odom'
                    marker.pose = marker_pose
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD  # TODO DELETE markers that aren't visible
                    marker.scale = Vector3(x=0.1, y=0.1, z=0.01)
                    marker.color = ColorRGBA(r=1., g=1., b=0., a=1.)
                    marker_array.markers.append(marker)
                    self.publish_tf(marker_pose, stamp, 'marker%d' % marker_id)
                self._rviz_markers_pub.publish(marker_array)

        cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = CameraTest()

    try:
        node.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C detected, shutting down")
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
