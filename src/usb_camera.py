#!/usr/bin/env python

import threading
import time

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2

# Grab frames from a USB camera and publish them as ROS messages at 1Hz for testing
# TODO Ctrl-C escalates to SIGTERM, perhaps due to threading
# TODO publish camera info


def now():
    cpu_time = time.time()
    sec = int(cpu_time)
    nanosec = int((cpu_time - sec) * 1000000)
    return Time(sec=sec, nanosec=nanosec)


class CameraTest(Node):

    def __init__(self):
        super().__init__('camera_test')

        self._image_pub = self.create_publisher(Image, 'image_raw')
        self._cv_bridge = CvBridge()

        self._video_thread = None
        self._stop_request = None

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

        frame_num = 0
        while not self._stop_request.isSet():
            ret, frame = cap.read()

            # Publish at 1Hz to reduce CPU usage
            frame_num += 1
            if frame_num == (30-1):
                frame_num = 0

                image_msg = self._cv_bridge.cv2_to_imgmsg(frame, 'bgr8')
                image_msg.header.stamp = now()
                self._image_pub.publish(image_msg)

            cv2.imshow('frame', frame)
            cv2.waitKey(1)

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
