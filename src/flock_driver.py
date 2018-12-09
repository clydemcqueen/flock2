#!/usr/bin/env python

import threading

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from flock2.msg import Flip, FlightData

import av
import cv2
import numpy as np
import tellopy

from video_node import VideoNode


class FlockDriver(VideoNode):

    FLIGHT_DATA_TOPIC = 'flight_data'

    CMD_VEL_TOPIC = 'cmd_vel'
    TAKEOFF_TOPIC = 'takeoff'
    LAND_TOPIC = 'land'
    FLIP_TOPIC = 'flip'

    def __init__(self):

        # Clyde's Tello camera, 960x720
        camera_matrix = np.array(
            [[921.170702, 0.000000, 459.904354],
             [0.000000, 919.018377, 351.238301],
             [0.000000, 0.000000, 1.000000]])
        distortion = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])

        # Approx covariance at 1 meter
        covariance_1m = np.zeros((6, 6))
        covariance_1m[0, 0] = 3e-4
        covariance_1m[1, 1] = 6e-3
        covariance_1m[2, 2] = 6e-3
        covariance_1m[3, 3] = 1e-4
        covariance_1m[4, 4] = 2e-3
        covariance_1m[5, 5] = 2e-3

        super().__init__('flock_driver', camera_matrix, distortion, covariance_1m)

        # ROS publishers
        self._flight_data_pub = self.create_publisher(FlightData, self.FLIGHT_DATA_TOPIC)

        # ROS subscriptions
        self.create_subscription(Twist, self.CMD_VEL_TOPIC, self._cmd_vel_callback)
        self.create_subscription(Empty, self.TAKEOFF_TOPIC, self._takeoff_callback)
        self.create_subscription(Empty, self.LAND_TOPIC, self._land_callback)
        self.create_subscription(Flip, self.FLIP_TOPIC, self._flip_callback)

        self._drone = None
        self._video_thread = None
        self._stop_request = None

        # Allocate messages
        self._flight_data_msg = FlightData()

    def connect(self):
        self.get_logger().info('trying to connect...')
        self._drone = tellopy.Tello()
        self._drone.set_loglevel(1)  # LOG_WARN only
        self._drone.connect()
        self._drone.wait_for_connection(120.0)
        self._drone.subscribe(self._drone.EVENT_FLIGHT_DATA, self._flight_data_callback)
        self.get_logger().info('connected')

        self._stop_request = threading.Event()
        self._video_thread = threading.Thread(target=self._video_worker)
        self._video_thread.start()

    def disconnect(self):
        self._drone.land()
        self._stop_request.set()
        self._video_thread.join(timeout=2)
        self._drone.quit()
        self._drone = None
        self.get_logger().info('disconnected')

    def _flight_data_callback(self, event, sender, data, **args):
        # Battery state
        self._flight_data_msg.battery_percent = data.battery_percentage
        self._flight_data_msg.estimated_flight_time_remaining = data.drone_fly_time_left / 10.
        self._flight_data_msg.battery_low = bool(data.battery_low)
        self._flight_data_msg.battery_lower = bool(data.battery_lower)
        if self._flight_data_msg.battery_lower:
            self.get_logger().warn('VERY LOW BATTERY')

        # Flight mode
        self._flight_data_msg.flight_mode = data.fly_mode

        # Flight time
        self._flight_data_msg.flight_time = data.fly_time / 10.

        # Very coarse velocity data
        self._flight_data_msg.east_speed = -1. if data.east_speed > 30000 else data.east_speed / 10.
        self._flight_data_msg.north_speed = -1. if data.north_speed > 30000 else data.north_speed / 10.
        self._flight_data_msg.ground_speed = -1. if data.ground_speed > 30000 else data.ground_speed / 10.

        # Altitude
        self._flight_data_msg.altitude = -1. if data.height > 30000 else data.height / 10.

        # Equipment status
        self._flight_data_msg.equipment = data.electrical_machinery_state
        self._flight_data_msg.high_temperature = bool(data.temperature_height)

        # Some state indicators?
        self._flight_data_msg.em_ground = bool(data.em_ground)
        self._flight_data_msg.em_sky = bool(data.em_sky)
        self._flight_data_msg.em_open = bool(data.em_open)

        # Publish what we have
        if self.count_subscribers(self.FLIGHT_DATA_TOPIC) > 0:
            self._flight_data_pub.publish(self._flight_data_msg)

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

    def _cmd_vel_callback(self, msg: Twist):
        self._drone.set_pitch(msg.linear.x)
        self._drone.set_roll(-msg.linear.y)  # Note sign flip
        self._drone.set_throttle(msg.linear.z)
        self._drone.set_yaw(-msg.angular.z)  # Note sign flip

    def _takeoff_callback(self, msg: Empty):
        self.get_logger().info('taking off')
        self._drone.takeoff()

    def _land_callback(self, msg: Empty):
        self.get_logger().info('landing')
        self._drone.land()

    def _flip_callback(self, msg: Flip):
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

    def _video_worker(self):
        # Get video stream, open in PyAV.
        container = av.container.open(self._drone.get_video_stream())

        # Decode h264
        self.get_logger().info('starting video pipeline')
        for frame in container.decode(video=0):

            # Convert PyAV frame => PIL image => OpenCV Mat
            color_mat = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)

            # Process frame
            self._process_frame(color_mat)

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
