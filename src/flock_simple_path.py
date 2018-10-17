#!/usr/bin/env python

import time

import numpy as np
import transformations as xf

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist, TransformStamped
from tf2_msgs.msg import TFMessage


def now():
    cpu_time = time.time()
    sec = int(cpu_time)
    nanosec = int((cpu_time - sec) * 1000000)
    return Time(sec=sec, nanosec=nanosec)


class TrajectoryHandler(object):

    def __init__(self):

        # set up the variables of interest
        self._positions = np.array([])  # list of the [N, E, D] positions for the trajectory in
        self._rel_times = np.array([])  # list of the relative times for the trajectory in decimal seconds

    def _load_trajectory(self, data):
        # - each row contains a trajectory point
        # - each row consists of the following 4 comma separated values:
        #    - relative time (seconds)
        #    - x position
        #    - y posiiton
        #    - z position

        # make the values relative to the first
        d = np.array(data)
        d = d - d[0, :]

        self._positions = d[:, 0]
        self._rel_times = d[:, 1:4]

    def get_next_point(self, inflight_time):
        # get the index of the trajectory with the closest time
        ind_min = np.argmin(np.abs(self._rel_times - inflight_time))

        # get the time of this point
        time_ref = self._rel_times[ind_min]

        # create the position and velocity commands,
        # which depends on if current time is before or after the trajectory point
        if inflight_time < time_ref:  # before the current point

            p0 = self._positions[ind_min - 1]
            p1 = self._positions[ind_min]

            t0 = self._rel_times[ind_min - 1]
            t1 = self._rel_times[ind_min]

        else:  # after the current point

            # now need to check if we are at the end of the file
            if ind_min >= len(self._positions) - 1:

                p0 = self._positions[ind_min]
                p1 = self._positions[ind_min]

                t0 = 0.0
                t1 = 1.0

            else:

                p0 = self._positions[ind_min]
                p1 = self._positions[ind_min + 1]

                t0 = self._rel_times[ind_min]
                t1 = self._rel_times[ind_min + 1]

        # compute the position command (interpolating between points)
        position_cmd = (p1 - p0) * (inflight_time - t0) / (t1 - t0) + p0

        # compute the velocity command based on the desired points and time
        velocity_cmd = (p1 - p0) / (t1 - t0)

        return position_cmd, velocity_cmd

    def is_trajectory_completed(self, inflight_time):
        return inflight_time > self._rel_times[-1]


class VelocityController(object):

    def __init__(self):

        # define all the gains that will be needed
        self._kp_pos = 0.4  # gain for lateral position error
        self._kp_alt = 0.3  # gain for altitude error
        self._kp_yaw = 0.2  # gain for yaw error

        # some limits to use
        self._v_max = 0.3       # the maximum horizontal velocity
        self._hdot_max = 0.4    # the maximum vertical velocity
        self._yawdot_max = 0.3  # the maximum yaw rate

    def lateral_position_control(self, pos_cmd, pos, vel_cmd):
        pos_error = pos_cmd[0:2] - pos[0:2]
        lateral_vel_cmd = self._kp_pos * pos_error + vel_cmd[0:2]
        lateral_vel_cmd = np.clip(lateral_vel_cmd, -self._v_max, self._v_max)
        return lateral_vel_cmd

    def altitude_control(self, alt_cmd, alt, hdot_cmd=0.0):
        hdot_cmd += self._kp_alt * (alt_cmd - alt)
        hdot_cmd = np.clip(hdot_cmd, -self._hdot_max, self._hdot_max)
        return hdot_cmd

    def yaw_control(self, yaw_cmd, alt, yawdot_cmd=0.0):
        yawdot_cmd += self._kp_yaw * (yaw_cmd - alt)
        yawdot_cmd = np.clip(yawdot_cmd, -self._yawdot_max, self._yawdot_max)
        return yawdot_cmd


class TrajectoryVelocityFlyer:

    def __init__(self):
        self.controller = VelocityController()
        self.trajectory = TrajectoryHandler()
        self.cmd_callback = None

    def start(self):
        pass

    def stop(self):
        pass

    def set_trajectory(self, waypoints):
        pass

    def cmd_subscription(self, cmd_callback):
        self.cmd_callback = cmd_callback

    def set_state(self, time, state):
        print(state)


class FlockSimplePath(Node):

    def __init__(self, flyer):
        super().__init__('flock_simple_path')
        self.flyer = flyer
        flyer.cmd_subscription(self.flyer_cmd_callback)

        # ROS publishers
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel')

        # ROS subscriptions
        self.create_subscription(TFMessage, '/tf', self.ros_tf_callback)

        self.get_logger().info('init complete')

    def start(self):
        pass

    def stop(self):
        pass

    def ros_tf_callback(self, msg):
        for transform in msg.transforms:  # type: TransformStamped
            if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_link':
                xlat = transform.transform.translation
                quat = transform.transform.rotation
                _, _, yaw = xf.euler_from_quaternion([quat.w, quat.x, quat.y, quat.z])
                state = np.array([xlat.x, xlat.y, xlat.z, yaw])
                self.flyer.set_state(now(), state)

    def flyer_cmd_callback(self, cmd):
        twist = Twist()
        print(cmd)


def get_trajectory(name):
    if name == 'lineX':
        return [
            [0.0, 0.0, 0.0, 0.0],
            [5.0, 1.0, 0.0, 0.0]
        ]
    else:
        return [
            [0.0, 0.0, 0.0, 0.0]
        ]


def main(args=None):
    rclpy.init(args=args)
    flyer = TrajectoryVelocityFlyer()
    flyer.set_trajectory(get_trajectory('lineX'))
    node = FlockSimplePath(flyer)

    try:
        node.start()
        flyer.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C detected, shutting down")
    finally:
        flyer.stop()
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
