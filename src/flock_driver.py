#!/usr/bin/env python

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, Empty
from tf2_msgs.msg import TFMessage
from flock2.msg import Flip, FlightData
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

import av
import cv2
import numpy
import tellopy

import util
import detect_aruco


class FlockDriver(Node):

    def __init__(self):
        super().__init__('flock_driver')

        # Clyde's Tello camera
        camera_matrix = numpy.array(
            [[921.170702, 0.000000, 459.904354],
             [0.000000, 919.018377, 351.238301],
             [0.000000, 0.000000, 1.000000]])
        distortion = numpy.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])

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

        # ROS publishers
        self._flight_data_pub = self.create_publisher(FlightData, 'flight_data')
        self._pose_pub = self.create_publisher(PoseStamped, 'pose')
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

        # ROS subscriptions
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback)
        self.create_subscription(Empty, 'takeoff', self.takeoff_callback)
        self.create_subscription(Empty, 'land', self.land_callback)
        self.create_subscription(Flip, 'flip', self.flip_callback)

        self._drone = None
        self._video_thread = None
        self._stop_request = None

        self.get_logger().info('init complete')

    def connect(self):
        self.get_logger().info('trying to connect...')
        self._drone = tellopy.Tello()
        self._drone.connect()
        self._drone.wait_for_connection(120.0)
        self._drone.subscribe(self._drone.EVENT_FLIGHT_DATA, self.flight_data_callback)
        self.get_logger().info('connected')

        self._stop_request = threading.Event()
        self._video_thread = threading.Thread(target=self.video_worker)
        self._video_thread.start()

    def disconnect(self):
        self._drone.land()
        self._stop_request.set()
        self._video_thread.join(timeout=2)
        self._drone.quit()
        self._drone = None
        self.get_logger().info('disconnected')

    def flight_data_callback(self, event, sender, data, **args):
        flight_data = FlightData()

        # Battery state
        flight_data.battery_percent = data.battery_percentage
        flight_data.estimated_flight_time_remaining = data.drone_fly_time_left / 10.
        flight_data.battery_low = bool(data.battery_low)
        flight_data.battery_lower = bool(data.battery_lower)

        # Flight mode
        flight_data.flight_mode = data.fly_mode

        # Flight time
        flight_data.flight_time = data.fly_time / 10.

        # Very coarse velocity data
        flight_data.east_speed = -1. if data.east_speed > 30000 else data.east_speed / 10.
        flight_data.north_speed = -1. if data.north_speed > 30000 else data.north_speed / 10.
        flight_data.ground_speed = -1. if data.ground_speed > 30000 else data.ground_speed / 10.

        # Altitude
        flight_data.altitude = -1. if data.height > 30000 else data.height / 10.

        # Equipment status
        flight_data.equipment = data.electrical_machinery_state
        flight_data.high_temperature = bool(data.temperature_height)

        # Some state indicators?
        flight_data.em_ground = bool(data.em_ground)
        flight_data.em_sky = bool(data.em_sky)
        flight_data.em_open = bool(data.em_open)

        # Publish what we have
        self._flight_data_pub.publish(flight_data)

        # Debugging: is there data here? Print nonzero values
        log = self.get_logger()
        if data.battery_state:
            log.info('battery_state is nonzero: %d' % data.battery_state)
        if data.drone_battery_left:
            log.info('drone_battery_left is nonzero: %d' % data.drone_battery_left)
        if data.camera_state:
            log.info('camera_state is nonzero: %d' % data.camera_state)
        if data.down_visual_state:
            log.info('down_visual_state is nonzero: %d' % data.down_visual_state)
        if data.drone_hover:
            log.info('drone_hover is nonzero: %d' % data.drone_hover)
        if data.factory_mode:
            log.info('factory_mode is nonzero: %d' % data.factory_mode)
        if data.front_in:
            log.info('front_in is nonzero: %d' % data.front_in)
        if data.front_lsc:
            log.info('front_lsc is nonzero: %d' % data.front_lsc)
        if data.front_out:
            log.info('front_out is nonzero: %d' % data.front_out)
        if data.gravity_state:
            log.info('gravity_state is nonzero: %d' % data.gravity_state)
        if data.imu_calibration_state:
            log.info('imu_calibration_state is nonzero: %d' % data.imu_calibration_state)
        if data.imu_state:
            log.info('imu_state is nonzero: %d' % data.imu_state)
        if data.outage_recording:
            log.info('outage_recording is nonzero: %d' % data.outage_recording)
        if data.power_state:
            log.info('power_state is nonzero: %d' % data.power_state)
        if data.pressure_state:
            log.info('pressure_state is nonzero: %d' % data.pressure_state)
        if data.throw_fly_timer:
            log.info('throw_fly_timer is nonzero: %d' % data.throw_fly_timer)
        if data.wind_state:
            log.info('wind_state is nonzero: %d' % data.wind_state)

    def cmd_vel_callback(self, msg):
        self._drone.set_pitch(msg.linear.x)
        self._drone.set_roll(-msg.linear.y)     # Note sign flip
        self._drone.set_throttle(msg.linear.z)
        self._drone.set_yaw(-msg.angular.z)     # Note sign flip

    def takeoff_callback(self, msg):
        self.get_logger().info('taking off')
        self._drone.takeoff()

    def land_callback(self, msg):
        self.get_logger().info('landing')
        self._drone.land()

    def flip_callback(self, msg):
        if msg.flip_command == Flip.FLIP_FORWARD:
            self._drone.flip_forward()
        elif msg.flip_command == Flip.FLIP_BACK:
            self._drone.flip_back()
        elif msg.flip_command == Flip.FLIP_LEFT:
            self._drone.flip_left()
        elif msg.flip_command == Flip.FLIP_RIGHT:
            self._drone.flip_right()
        elif msg.flip_command == Flip.FLIP_FORWARDLEFT:
            self._drone.flip_forwardleft()
        elif msg.flip_command == Flip.FLIP_FORWARDRIGHT:
            self._drone.flip_forwardright()
        elif msg.flip_command == Flip.FLIP_BACKLEFT:
            self._drone.flip_backleft()
        elif msg.flip_command == Flip.FLIP_BACKRIGHT:
            self._drone.flip_backright()

    def video_worker(self):
        # Get video stream, open in PyAV.
        container = av.container.open(self._drone.get_video_stream())

        # Decode h264
        self.get_logger().info('starting video pipeline')
        for frame in container.decode(video=0):
            stamp = util.now()

            # Convert PyAV frame => PIL image => OpenCV Mat
            color_mat = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)

            # Detect markers
            color_mat, drone_pose, marker_poses = self._detector.detect(self.get_logger(), color_mat)

            # Optionally publish images
            if self._publish_images:
                image_msg = self._cv_bridge.cv2_to_imgmsg(color_mat, 'bgr8')
                image_msg.header.stamp = stamp
                self._image_pub.publish(image_msg)
            else:
                cv2.imshow('usb_camera', color_mat)
                cv2.waitKey(1)

            # Publish drone pose
            if drone_pose is not None:
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = util.now()
                pose_stamped.header.frame_id = 'base_link'
                pose_stamped.pose = drone_pose
                self._pose_pub.publish(pose_stamped)

            # Publish transforms and rviz markers
            if drone_pose is not None and marker_poses is not None:
                tf2 = TFMessage()
                tf2.transforms.append(util.pose_to_transform(drone_pose, stamp, 'odom', 'base_link'))
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
                    tf2.transforms.append(util.pose_to_transform(marker_pose, stamp, 'odom', 'marker%d' % marker_id))
                self._tf_pub.publish(tf2)
                self._rviz_markers_pub.publish(marker_array)

            # Check for normal shutdown
            if self._stop_request.isSet():
                return


def main(args=None):
    rclpy.init(args=args)
    node = FlockDriver()

    try:
        node.connect()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C detected, shutting down")
        node.disconnect()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
