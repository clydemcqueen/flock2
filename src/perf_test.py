#!/usr/bin/env python

import time

from cProfile import Profile
from pstats import Stats

import rclpy
from nav_msgs.msg import Odometry

import cv2
import numpy as np
import transformations as xf

import filter
import usb_camera

SIGMA = 3e-4


def filter_test():
    num = 2000
    rclpy.init()
    filter_node = filter.Filter()
    sim_time = start_time = time.time()

    odom_msg = Odometry()
    odom_msg.header.frame_id = 'odom'
    odom_msg.child_frame_id = 'base_link'
    odom_msg.pose.pose.position.x = np.random.normal(0, SIGMA)
    odom_msg.pose.pose.position.y = np.random.normal(0, SIGMA)
    odom_msg.pose.pose.position.z = np.random.normal(0, SIGMA)
    xf.quaternion_from_euler(*np.random.normal(0, SIGMA, 3))
    odom_msg.pose.covariance = [
        SIGMA, 0., 0., 0., 0., 0.,
        0., SIGMA, 0., 0., 0., 0.,
        0., 0., SIGMA, 0., 0., 0.,
        0., 0., 0., SIGMA, 0., 0.,
        0., 0., 0., 0., SIGMA, 0.,
        0., 0., 0., 0., 0., SIGMA,
    ]

    for i in range(num):
        odom_msg.header.stamp.sec = int(sim_time)
        odom_msg.header.stamp.nanosec = int((sim_time - odom_msg.header.stamp.sec) * 1000000)

        filter_node._odom_callback(odom_msg)
        sim_time += 1. / 30

    elapsed_sim_time = sim_time - start_time
    elapsed_real_time = time.time() - start_time
    ratio = elapsed_sim_time / elapsed_real_time
    print('sim time %f, real time %f, ratio %f' % (elapsed_sim_time, elapsed_real_time, ratio))


def camera_test():
    num = 2000
    rclpy.init()
    camera_node = usb_camera.CameraTest()
    sim_time = start_time = time.time()

    cap = cv2.VideoCapture(0)
    print('%d x %d' % (cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

    ret, frame = cap.read()
    if not ret:
        print('no image')
        return

    for i in range(num):
        camera_node._process_frame(frame)
        sim_time += 1. / 30

    cap.release()
    cv2.destroyAllWindows()

    elapsed_sim_time = sim_time - start_time
    elapsed_real_time = time.time() - start_time
    ratio = elapsed_sim_time / elapsed_real_time
    print('sim time %f, real time %f, ratio %f' % (elapsed_sim_time, elapsed_real_time, ratio))


def whole_node_test():
    num = 2000
    rclpy.init()

    c = usb_camera.CameraTest()
    c.start()
    f = filter.Filter()

    for i in range(num):
        rclpy.spin_once(c, timeout_sec=1./100)
        rclpy.spin_once(f, timeout_sec=1./100)

    c.stop()
    c.destroy_node()
    f.destroy_node()
    rclpy.shutdown()


def main():
    profiler = Profile()
    profiler.runcall(whole_node_test)
    stats = Stats(profiler)
    stats.strip_dirs()
    stats.sort_stats('cumulative')
    stats.print_stats(20)


if __name__ == '__main__':
    main()
