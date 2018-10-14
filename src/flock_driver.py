#!/usr/bin/env python

import threading
import time

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from flock2.msg import Flip, FlightData
from cv_bridge import CvBridge

import av
import cv2
import numpy
import tellopy


def now():
    cpu_time = time.time()
    sec = int(cpu_time)
    nanosec = int((cpu_time - sec) * 1000000)
    return Time(sec=sec, nanosec=nanosec)


class FlockDriver(Node):

    def __init__(self):
        super().__init__('flock_driver')

        # ROS publishers
        self._flight_data_pub = self.create_publisher(FlightData, 'flight_data')
        self._image_pub = self.create_publisher(Image, 'image_raw')

        # ROS subscriptions
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback)
        self.create_subscription(Empty, 'takeoff', self.takeoff_callback)
        self.create_subscription(Empty, 'land', self.land_callback)
        self.create_subscription(Flip, 'flip', self.flip_callback)

        # ROS OpenCV wrapper
        self._cv_bridge = CvBridge()

        self._drone = None
        self._video_thread = None
        self._stop_request = None

        self.get_logger().info('init complete')

    def connect(self):
        self.get_logger().info('trying to connect...')
        self._drone = tellopy.Tello()
        self._drone.connect()
        self._drone.wait_for_connection(90.0)
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
        if data.battery_low:
            log.info('battery_low is nonzero: %d' % data.battery_low)
        if data.battery_lower:
            log.info('battery_lower is nonzero: %d' % data.battery_lower)
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
        frame_num = 0
        for frame in container.decode(video=0):
            # Publish at 1Hz to reduce CPU load
            frame_num += 1
            if frame_num == (30-1):
                frame_num = 0

                # Convert PyAV frame => PIL image => OpenCV Mat
                color_mat = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)

                # Convert OpenCV Mat => ROS Image message and publish
                image_msg = self._cv_bridge.cv2_to_imgmsg(color_mat, 'bgr8')
                image_msg.header.stamp = now()
                self._image_pub.publish(image_msg)

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
