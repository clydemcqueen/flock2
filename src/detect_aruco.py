#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion, Transform, TransformStamped, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

import cv2
import numpy
import transformations

# Transformation notation:
# Tst === T_source_target
# vector_target = T_source_target * vector_source


def rvec_and_tvec_to_matrix(rvec, tvec):
    """Rodrigues rotation and translation vector to 4x4 matrix"""
    t_matrix = transformations.translation_matrix(tvec)
    R, _ = cv2.Rodrigues(rvec)
    r_matrix = transformations.identity_matrix()
    r_matrix[:3, :3] = R
    return numpy.dot(t_matrix, r_matrix)


def tf_to_matrix(ros_transform):
    """ROS transform to 4x4 matrix"""
    t, q = ros_transform
    t_matrix = transformations.translation_matrix(t)
    r_matrix = transformations.quaternion_matrix(q)
    return numpy.dot(t_matrix, r_matrix)


def matrix_to_tf(T):
    """4x4 matrix to ROS transform"""
    t = transformations.translation_from_matrix(T)
    q = transformations.quaternion_from_matrix(T)
    return t, q


def pose_to_matrix(p):
    """geometry_msgs.msg.Pose to 4x4 matrix"""
    t_matrix = transformations.translation_matrix([p.position.x, p.position.y, p.position.z])
    r_matrix = transformations.quaternion_matrix([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
    return numpy.dot(t_matrix, r_matrix)


def matrix_to_pose(matrix):
    """4x4 matrix to geometry_msgs.msg.Pose"""
    t, q = matrix_to_tf(matrix)
    return Pose(position=Point(x=t[0], y=t[1], z=t[2]), orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))


def tf_to_pose(ros_transform):
    """ROS transform to geometry_msgs.msg.Pose"""
    t, q = ros_transform
    return Pose(position=Point(x=t[0], y=t[1], z=t[2]), orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))


class DetectArUco(Node):

    def __init__(self):
        super().__init__('detect_aruco')

        # Clyde's Tello camera
        camera_matrix1 = numpy.array(
            [[921.170702, 0.000000, 459.904354],
             [0.000000, 919.018377, 351.238301],
             [0.000000, 0.000000, 1.000000]])
        distortion1 = numpy.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])

        # Clyde's USB camera
        camera_matrix2 = numpy.array(
            [[460.465192, 0.000000, 350.820241],
             [0.000000, 459.337700, 251.915294],
             [0.000000, 0.000000, 1.000000]])
        distortion2 = numpy.array([0.006521, -0.006982, 0.001500, 0.000761, 0.000000])

        # TODO pull camera info from a camera info message
        self._camera_matrix = camera_matrix1
        self._distortion = distortion1

        # ArUco data -- we're using 6x6 ArUco images
        self._aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self._aruco_parameters = cv2.aruco.DetectorParameters_create()

        # Markers are 18cm x 18cm
        self._marker_length = 0.18

        # ID of the first marker we see
        self._first_marker_id = -1

        # First marker pose
        self._first_marker_pose = tf_to_pose(([0., 0., 1.], [0.5, -0.5, -0.5, 0.5]))
        self._Tom = transformations.inverse_matrix(pose_to_matrix(self._first_marker_pose))

        # Tcb transform
        self._Tcb = tf_to_matrix(([0.035, 0., 0.], [-0.5, -0.5, 0.5, 0.5]))

        # Publications
        self._tf_pub = self.create_publisher(TFMessage, '/tf')
        self._image_pub = self.create_publisher(Image, 'image_marked')
        self._rviz_markers_pub = self.create_publisher(MarkerArray, 'rviz_markers')

        # Subscriptions
        self.create_subscription(Image, "image_raw", self.image_callback)

        # ROS OpenCV bridge
        self._cv_bridge = CvBridge()

    def publish_tf(self, t, q, stamp, child, parent):
        geometry_msg = TransformStamped()
        geometry_msg.header.frame_id = parent
        geometry_msg.header.stamp = stamp
        geometry_msg.child_frame_id = child
        geometry_msg.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        geometry_msg.transform.translation = Vector3(x=t[0], y=t[1], z=t[2])
        tf2_msg = TFMessage()
        tf2_msg.transforms.append(geometry_msg)
        self._tf_pub.publish(tf2_msg)

    def publish_tf_pose(self, p, stamp, child, parent):
        t = Vector3(x=p.position.x, y=p.position.y, z=p.position.z)
        geometry_msg = TransformStamped()
        geometry_msg.header.frame_id = parent
        geometry_msg.header.stamp = stamp
        geometry_msg.child_frame_id = child
        geometry_msg.transform = Transform(translation=t, rotation=p.orientation)
        tf2_msg = TFMessage()
        tf2_msg.transforms.append(geometry_msg)
        self._tf_pub.publish(tf2_msg)

    def image_callback(self, msg):
        # Image time
        stamp = msg.header.stamp

        # Convert ROS image ==> OpenCV Mat
        color_mat = self._cv_bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Color => gray for detection
        gray_mat = cv2.cvtColor(color_mat, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = cv2.aruco.detectMarkers(gray_mat, self._aruco_dict, parameters=self._aruco_parameters)

        # Stop if no markers were detected
        if ids is None:
            return

        # Draw borders on the color image
        color_mat = cv2.aruco.drawDetectedMarkers(color_mat, corners)

        # Publish the marked up image
        self._image_pub.publish(self._cv_bridge.cv2_to_imgmsg(color_mat, 'bgr8'))

        if self._first_marker_id < 0:
            # Grab the first marker we see
            self._first_marker_id = ids[0][0]
            self.get_logger().info('First marker has id %d' % self._first_marker_id)
        else:
            # Stop if the first marker wasn't detected
            found_first_marker = False
            for index in range(len(ids)):
                if ids[index][0] == self._first_marker_id:
                    found_first_marker = True
                    break
            if not found_first_marker:
                return

        # Compute transformations, each is marker_frame => camera_frame
        rvecs, tvecs, object_points = cv2.aruco.estimatePoseSingleMarkers(corners, self._marker_length, self._camera_matrix, self._distortion)

        # Compute the pose of the drone
        Tbo = transformations.identity_matrix()
        for index in range(len(ids)):
            if ids[index][0] == self._first_marker_id:
                # Tob = Tcb * Tmc * Tom
                Tmc = rvec_and_tvec_to_matrix(rvecs[index][0], tvecs[index][0])

                Tob = self._Tcb.dot(Tmc).dot(self._Tom)

                # sendTransform expects child=target and parent=source
                # We can't flip source and target because we want odom to be the parent of base_link
                # Instead, invert Tob to get Tbo
                Tbo = transformations.inverse_matrix(Tob)
                t, q = matrix_to_tf(Tbo)
                self.publish_tf(t, q, stamp, child='base_link', parent='odom')
                break

        # Compute the pose of the other markers, and publish a MarkerArray message for display in rviz
        marker_array = MarkerArray()
        for index in range(len(ids)):
            marker = Marker()
            marker.id = int(ids[index][0])
            marker.header.frame_id = 'odom'
            if ids[index][0] == self._first_marker_id:
                # This one is easy
                marker.pose = self._first_marker_pose
            else:
                # Compute the pose of this marker
                # Tmo = Tbo * Tcb * Tmc
                Tmc = rvec_and_tvec_to_matrix(rvecs[index][0], tvecs[index][0])
                Tmo = Tbo.dot(self._Tcb).dot(Tmc)
                marker.pose = matrix_to_pose(Tmo)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD  # TODO DELETE markers that aren't visible
            marker.scale = Vector3(x=0.1, y=0.1, z=0.01)
            marker.color = ColorRGBA(r=1., g=1., b=0., a=1.)
            marker_array.markers.append(marker)

            self.publish_tf_pose(marker.pose, stamp, child='marker' + str(marker.id), parent='odom')

        self._rviz_markers_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = DetectArUco()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C detected, shutting down")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
