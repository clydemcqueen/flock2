from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

import cv2
import numpy as np

import util
import detect_aruco


class VideoNode(Node):

    IMAGE_TOPIC = 'image_marked'
    ODOM_TOPIC = 'odom'
    MARKER_TOPIC = 'rviz_markers'
    TF_TOPIC = '/tf'

    WORLD_FRAME_ID = 'odom'
    BASE_FRAME_ID = 'base_link'

    def __init__(self, name: str, camera_matrix: np.ndarray, distortion: np.ndarray, covariance_1m: np.ndarray):
        super().__init__(name)

        # ArUco detector
        self._detector = detect_aruco.DetectArUco(camera_matrix, distortion)

        # Covariance at 1m
        self._covariance_1m = covariance_1m

        # Send some data "best effort"
        # Set "unreliable" in rviz2
        # Sadly these messages don't show up in `ros2 topic echo`
        self._best_effort = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)

        # Publishers
        self._odom_pub = self.create_publisher(Odometry, self.ODOM_TOPIC, qos_profile=self._best_effort)
        self._rviz_markers_pub = self.create_publisher(MarkerArray, self.MARKER_TOPIC, qos_profile=self._best_effort)
        self._tf_pub = self.create_publisher(TFMessage, self.TF_TOPIC)

        # Optionally publish images
        self._publish_images = False
        if self._publish_images:
            self._image_pub = self.create_publisher(Image, self.IMAGE_TOPIC, qos_profile=self._best_effort)
            self._cv_bridge = CvBridge()
        else:
            self._image_pub = None
            self._cv_bridge = None

        self._video_thread = None
        self._stop_request = None

        # Allocate messages
        self._odom_msg = Odometry()
        self._odom_msg.header.frame_id = self.WORLD_FRAME_ID
        self._odom_msg.child_frame_id = self.BASE_FRAME_ID
        self._tf_msg = TFMessage()
        self._marker_array_msg = MarkerArray()

    def _process_frame(self, frame):
        stamp = util.now()

        # Detect markers
        frame, drone_pose, marker_poses = self._detector.detect(self.get_logger(), frame)

        # Publish images
        if self._publish_images:
            if self.count_subscribers(self.IMAGE_TOPIC) > 0:
                image_msg = self._cv_bridge.cv2_to_imgmsg(frame, 'bgr8')
                image_msg.header.stamp = stamp
                self._image_pub.publish(image_msg)
        else:
            cv2.imshow('video', frame)
            cv2.waitKey(1)

        # Publish odometry
        if drone_pose is not None and self.count_subscribers(self.ODOM_TOPIC) > 0:
            # Covariance scales w/ distance**3
            covariance = (self._covariance_1m * ((1 + (-drone_pose.position.x - 1)) ** 3)).flatten().tolist()

            self._odom_msg.header.stamp = stamp
            self._odom_msg.pose.pose = drone_pose
            self._odom_msg.pose.covariance = covariance
            # Twist is 0
            self._odom_pub.publish(self._odom_msg)

        # Publish rviz markers
        if marker_poses is not None and self.count_subscribers(self.MARKER_TOPIC) > 0:
            self._marker_array_msg.markers.clear()
            for marker_id, marker_pose in marker_poses.items():
                marker = Marker()
                marker.id = marker_id
                marker.header.frame_id = self.WORLD_FRAME_ID
                marker.pose = marker_pose
                marker.type = Marker.CUBE
                marker.action = Marker.ADD  # TODO DELETE markers that aren't visible
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.01
                marker.color.r = 1.
                marker.color.g = 1.
                marker.color.b = 0.
                marker.color.a = 1.
                self._marker_array_msg.markers.append(marker)
            self._rviz_markers_pub.publish(self._marker_array_msg)

        # Publish transforms
        if marker_poses is not None and self.count_subscribers(self.TF_TOPIC) > 0:
            self._tf_msg.transforms.clear()
            for marker_id, marker_pose in marker_poses.items():
                transform_stamped = TransformStamped()
                transform_stamped.header.frame_id = self.WORLD_FRAME_ID
                transform_stamped.header.stamp = stamp
                transform_stamped.child_frame_id = 'marker%d' % marker_id
                util.copy_pose_to_transform(marker_pose, transform_stamped.transform)
                self._tf_msg.transforms.append(transform_stamped)
            self._tf_pub.publish(self._tf_msg)
