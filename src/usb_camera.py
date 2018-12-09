#!/usr/bin/env python

import threading

import rclpy

import cv2
import numpy as np

from video_node import VideoNode

# Grab frames from a USB camera for testing


class CameraTest(VideoNode):

    def __init__(self):

        # Clyde's USB camera, SONY ICX810DKV or ICX811DKV, 6mm diagonal
        # Driver provides 640x480, assume pixels are 0.0076mm square, f=3.4mm
        camera_matrix = np.array(
            [[460.465192, 0.000000, 350.820241],
             [0.000000, 459.337700, 251.915294],
             [0.000000, 0.000000, 1.000000]])
        distortion = np.array([0.006521, -0.006982, 0.001500, 0.000761, 0.000000])

        # Approx covariance at 1 meter
        covariance_1m = np.zeros((6, 6))
        covariance_1m[0, 0] = 3e-4
        covariance_1m[1, 1] = 6e-3
        covariance_1m[2, 2] = 6e-3
        covariance_1m[3, 3] = 1e-4
        covariance_1m[4, 4] = 2e-3
        covariance_1m[5, 5] = 2e-3

        super().__init__('camera_test', camera_matrix, distortion, covariance_1m)

    def start(self):
        self.get_logger().info("starting video thread")
        self._stop_request = threading.Event()
        self._video_thread = threading.Thread(target=self._video_worker)
        self._video_thread.start()

    def stop(self):
        self.get_logger().info("stopping video thread")
        self._stop_request.set()
        self._video_thread.join(timeout=2)
        self._stop_request = None
        self._video_thread = None

    def _video_worker(self):
        cap = cv2.VideoCapture(0)
        self.get_logger().info('%d x %d' % (cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

        while not self._stop_request.isSet():
            ret, frame = cap.read()
            if ret:
                self._process_frame(frame)

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
