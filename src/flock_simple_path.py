#!/usr/bin/env python

import time
from enum import Enum

import numpy as np
import transformations as xf

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Empty
from tf2_msgs.msg import TFMessage

import smooth_path_4poly_2min


def now():
    cpu_time = time.time()
    sec = int(cpu_time)
    nanosec = int((cpu_time - sec) * 1000000)
    return Time(sec=sec, nanosec=nanosec)


def ros_time_to_time(ros_time: Time) -> float:
    return ros_time.sec + ros_time.nanosec / 1000000.0


class TrajectoryHandler(object):

    def __init__(self):

        # set up the variables of interest
        self._rel_times = np.array([])  # list of the relative times for the trajectory in decimal seconds
        self._positions = np.array([])  # list of the positions for the trajectory in

        self._use_waypoints = False
        self._path_3d = None

        self._repeat = False
        self._stabilize_sec = 0.

    def set_waypoints(self, data, repeat, stabilize_sec):
        # - each row contains a trajectory point
        # - each row consists of the following 4 comma separated values:
        #    - relative time (seconds)
        #    - x position
        #    - y posiiton
        #    - z position
        d = np.array(data)
        self._rel_times = d[:, 0]
        self._positions = d[:, 1:4]

        self._path_3d = smooth_path_4poly_2min.Path3d(self._rel_times, self._positions)

        self._repeat = repeat
        self._stabilize_sec = stabilize_sec

    def is_trajectory_completed(self, inflight_time):
        return (inflight_time + self._stabilize_sec > self._rel_times[-1]) if not self._repeat else False

    def get_point(self, inflight_time):
        # check time range
        inflight_time -= self._stabilize_sec

        if inflight_time <= 0:
            return self._positions[0], np.zeros(3)

        elif self._repeat:
            _, inflight_time = divmod(inflight_time, self._rel_times[-1])

        elif inflight_time >= self._rel_times[-1]:
            return self._positions[-1], np.zeros(3)

        return self._get_waypoint(inflight_time) if self._use_waypoints else self._get_smoothpoint(inflight_time)

    def _get_waypoint(self, inflight_time):
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

        # compute the position target (interpolating between points)
        position_target = (p1 - p0) * (inflight_time - t0) / (t1 - t0) + p0

        # compute the velocity target based on the desired points and time
        velocity_target = (p1 - p0) / (t1 - t0)

        return position_target, velocity_target

    def _get_smoothpoint(self, inflight_time):
        return self._path_3d.calc_y_and_y_dot(inflight_time)


class VelocityController(object):

    def __init__(self):

        # define all the gains that will be needed
        self._kp_pos = 1.0  # gain for lateral position error
        self._kp_alt = 1.0  # gain for altitude error
        self._kp_yaw = 1.0  # gain for yaw error

        # some limits to use
        self._v_max = 0.4       # the maximum horizontal velocity
        self._hdot_max = 0.4    # the maximum vertical velocity
        self._yawdot_max = 0.8  # the maximum yaw rate

    def lateral_position_control(self, pos_cmd, pos, vel_target=np.array((0., 0.))):
        pos_error = pos_cmd - pos
        lateral_vel_cmd = self._kp_pos * pos_error + vel_target
        lateral_vel_cmd = np.clip(lateral_vel_cmd, -self._v_max, self._v_max)
        print(
            ("pos_cmd:{0:7.3f},{1:7.3f}; pos:{2:7.3f},{3:7.3f}; pos_error:{4:7.3f},{5:7.3f}; " +
                "vel_target:{6:7.3f},{7:7.3f}; lateral_vel_cmd:{8:7.3f},{9:7.3f}")
            .format(pos_cmd[0], pos_cmd[1], pos[0], pos[1], pos_error[0], pos_error[1],
                    vel_target[0], vel_target[1], lateral_vel_cmd[0], lateral_vel_cmd[1]))
        return lateral_vel_cmd

    def altitude_control(self, alt_cmd, alt, hdot_target=0.0):
        alt_error = alt_cmd - alt
        hdot_cmd = self._kp_alt * alt_error + hdot_target
        hdot_cmd = np.clip(hdot_cmd, -self._hdot_max, self._hdot_max)
        return hdot_cmd

    def yaw_control(self, yaw_cmd, yaw, yawdot_target=0.0):
        yaw_error = yaw_cmd - yaw
        if yaw_error > np.pi:
            yaw_error -= 2.0 * np.pi
        elif yaw_error < -np.pi:
            yaw_error += 2.0 * np.pi

        yawdot_cmd = self._kp_yaw * yaw_error + yawdot_target
        yawdot_cmd = np.clip(yawdot_cmd, -self._yawdot_max, self._yawdot_max)
        # print(
        #     ("yaw_cmd:{0:7.3f}; yaw:{1:7.3f}; yaw_error:{2:7.3f}; " +
        #         "yawdot_target:{3:7.3f}; yawdot_cmd:{4:7.3f}")
        #     .format(yaw_cmd, yaw, yaw_error,
        #             yawdot_target, yawdot_cmd))
        return yawdot_cmd


class TrajectoryVelocityFlyer:
    class States(Enum):
        INIT = 0
        WAITING = 1
        LAUNCHING = 2
        RUNNING = 3
        LANDING = 4
        DONE = 5

    def __init__(self):
        self._controller = VelocityController()
        self._trajectory = TrajectoryHandler()
        self._callback_node = None
        self._state = self.States.INIT

        self._lag_limit_sec = 0.25
        self._launch_limit_sec = 10.0
        self._already_flying = True  # don't launch and land

        self._start_msg_time = 0.0
        self._start_drone_state = np.zeros(4)
        self._watchdog_clear = False

    def start(self):
        if self._state == self.States.INIT:
            self._state = self.States.WAITING

    def stop(self):
        assert self._callback_node

        if self._state == self.States.LAUNCHING or \
                self._state == self.States.RUNNING:
            # for now just tell the drone to stop (do not try to land) if it is flying.
            self._call_flyer_cmd_callback(np.zeros(4))

        last_state = self._state
        self._state = self.States.DONE

        # tell the node that we are stopping.
        if last_state != self.States.DONE:
            self._call_flyer_stoping_callback()

    def set_waypoints(self, waypoints, repeat=False, stabilize_sec=0.0):
        self._trajectory.set_waypoints(waypoints, repeat, stabilize_sec)

    def set_callback_node(self, callback_node):
        self._callback_node = callback_node

    def new_drone_state(self, msg_time, drone_state):
        self._watchdog_clear = True

        # Certain states don't process this message
        if self._state == self.States.INIT or \
                self._state == self.States.LANDING or \
                self._state == self.States.DONE:
            return

        if self._state == self.States.WAITING:
            # Wait for the lag to get small before starting
            if self._no_lag(msg_time):
                assert self._callback_node
                self._launch(msg_time, drone_state)
            return

        # if we stop getting state messages, then stop.
        if not self._no_lag(msg_time):
            self._call_flyer_log_info_callback('too much lag in state messages')
            self.stop()
            return

        # Launching rises to the height of the first way point or a fixed amount of time
        if self._state == self.States.LAUNCHING:
            pos_target, _ = self._trajectory.get_point(0.0)
            if drone_state[2] >= pos_target[2] or \
                    msg_time - self._start_msg_time > self._launch_limit_sec:
                self._start_msg_time = msg_time
                self._start_drone_state = drone_state
                self._state = self.States.RUNNING
            return

        # process the RUNNING state:
        if self._process_drone_state(msg_time, drone_state):
            self._land()

    def timer_fired(self):
        # if the clear flag is false, then we haven't had
        # a state message in a while so stop
        if not self._watchdog_clear:
            if self._state == self.States.LAUNCHING or \
                    self._state == self.States.RUNNING:
                self._call_flyer_log_info_callback('too much time between state messages')
                self.stop()
                return
        self._watchdog_clear = False

    def _no_lag(self, msg_time: float) -> bool:
        # Return True if the lag between when the message was sent and
        # now is not too long
        now_time = time.time()
        return now_time - msg_time < self._lag_limit_sec

    def _process_drone_state(self, msg_time, drone_state):
        # get the current in flight time
        rel_time = msg_time - self._start_msg_time

        # get the target position and velocity from the trajectory (in world frame)
        pos_target, vel_target = self._trajectory.get_point(rel_time)

        # run the controller for position (position controller -> to velocity command)
        vel_cmd = np.zeros(4)
        vel_cmd[0:2] = self._controller.lateral_position_control(pos_target[0:2], drone_state[0:2], vel_target[0:2])
        vel_cmd[2] = self._controller.altitude_control(pos_target[2], drone_state[2], vel_target[2])

        # The drone should yaw toward the origin. Figure out the yaw and raw_dot targets and run the yaw controller
        yaw_target = np.arctan2(-pos_target[1], -pos_target[0])
        vel_target_half = 0.5 * vel_target
        yaw_dot_target = np.arctan2(-pos_target[1] + vel_target_half[1], -pos_target[0] + vel_target_half[0]) - \
            np.arctan2(-pos_target[1] - vel_target_half[1], -pos_target[0] - vel_target_half[0])
        vel_cmd[3] = self._controller.yaw_control(yaw_target, drone_state[3], -yaw_dot_target)

        # The vel_cmd should be in the drone frame. Rotate the command velocities by the yaw.
        yaw = drone_state[3]
        vel_cmd_drone = np.copy(vel_cmd)
        vel_cmd_drone[0] = np.cos(yaw) * vel_cmd[0] + np.sin(yaw) * vel_cmd[1]
        vel_cmd_drone[1] = -np.sin(yaw) * vel_cmd[0] + np.cos(yaw) * vel_cmd[1]

        # print('pos_target:', pos_target, 'vel_target:',
        #   vel_target, 'vel_cmd', vel_cmd, 'vel_cmd_drone', vel_cmd_drone)

        # send the velocity command to the drone
        self._call_flyer_cmd_callback(vel_cmd_drone)

        # check for the end of the trajectory
        return self._trajectory.is_trajectory_completed(rel_time)

    def _launch(self, msg_time, drone_state):
        self._call_flyer_log_info_callback('launching')
        self._start_msg_time = msg_time
        self._start_drone_state = drone_state
        self._state = self.States.RUNNING if self._already_flying else self.States.LAUNCHING
        if not self._already_flying:
            self._call_flyer_takeoff_callback()

    def _land(self):
        self._call_flyer_log_info_callback('landing')
        if not self._already_flying:
            self._state = self.States.LANDING
            self._call_flyer_land_callback()
        self.stop()

    def _call_flyer_cmd_callback(self, vel_cmd):
        self._callback_node.flyer_cmd_callback(vel_cmd)

    def _call_flyer_stoping_callback(self):
        self._callback_node.flyer_stopping_callback()

    def _call_flyer_takeoff_callback(self):
        self._callback_node.flyer_takeoff_callback()

    def _call_flyer_land_callback(self):
        self._callback_node.flyer_land_callback()

    def _call_flyer_log_info_callback(self, msg):
        self._callback_node.flyer_log_info_callback(msg)


class FlockSimplePath(Node):

    class States(Enum):
        INIT = 0
        RUNNING = 1
        DONE = 2

    def __init__(self, flyer):
        super().__init__('flock_simple_path')
        self._flyer = flyer
        flyer.set_callback_node(self)

        self._state = self.States.INIT

        # ROS publishers
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel')
        self._takeoff_pub = self.create_publisher(Empty, 'takeoff')
        self._land_pub = self.create_publisher(Empty, 'land')

        # ROS subscriptions
        self.create_subscription(Empty, 'start_mission', self._ros_start_callback)
        self.create_subscription(Empty, 'stop_mission', self._ros_stop_callback)
        self.create_subscription(TFMessage, '/tf', self._ros_tf_callback)

        # Timer
        timer_period_sec = 1.
        self.create_timer(timer_period_sec, self._ros_timer_callback)

        self.get_logger().info('init complete')

    def start(self):
        if self._state == self.States.INIT:
            self._state = self.States.RUNNING
            self._flyer.start()

    def stop(self):
        if self._state == self.States.RUNNING:
            self._flyer.stop()
        self._state = self.States.DONE

    def _ros_start_callback(self, msg):
        self.get_logger().info('starting mission')
        self.start()

    def _ros_stop_callback(self, msg):
        self.get_logger().info('stopping mission')
        self.stop()

    def _ros_tf_callback(self, msg):
        if self._state == self.States.RUNNING:
            # only process tf messages for the drone position
            for transform in msg.transforms:  # type: TransformStamped
                if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_link':
                    xlat = transform.transform.translation
                    quat = transform.transform.rotation
                    _, _, yaw = xf.euler_from_quaternion([quat.w, quat.x, quat.y, quat.z])
                    state = np.array([xlat.x, xlat.y, xlat.z, yaw])
                    self._flyer.new_drone_state(ros_time_to_time(transform.header.stamp), state)

    def _ros_timer_callback(self):
        if self._state == self.States.RUNNING:
            self._flyer.timer_fired()

    def flyer_cmd_callback(self, vel_cmd):
        if self._state == self.States.RUNNING:
            # The commands coming from the flyer are a rate. How these translate
            # into drone commands is a mystery. For now pass the values on through.
            twist = Twist()
            twist.linear.x = vel_cmd[0]
            twist.linear.y = vel_cmd[1]
            twist.linear.z = vel_cmd[2]
            twist.angular.z = vel_cmd[3]

            self._cmd_vel_pub.publish(twist)

    def flyer_stopping_callback(self):
        # flyer has decided to stop.
        self._state = self.States.DONE

    def flyer_takeoff_callback(self):
        if self._state == self.States.RUNNING:
            self._takeoff_pub.publish(Empty())

    def flyer_land_callback(self):
        if self._state == self.States.RUNNING:
            self._land_pub.publish(Empty())

    def flyer_log_info_callback(self, msg):
        self.get_logger().info(msg)


class Figure:
    def __init__(self, speed, origin, scale):
        self._speed = speed
        self._origin = np.array(origin)
        self._scale = scale

    def _abs_pos(self, rel_pos):
        return rel_pos * self._scale + self._origin

    @staticmethod
    def _elem(t, abs_pos):
        return [t, abs_pos[0], abs_pos[1], abs_pos[2]]

    def _gen(self, func):
        curr_t = 0.0
        curr_pos = np.zeros(3)
        last_abs_pos = self._abs_pos(curr_pos)
        yield self._elem(0., last_abs_pos)
        for e in func(self):
            this_pos = curr_pos
            for s, pos in e:
                rel_pos = np.array(pos) + this_pos
                abs_pos = self._abs_pos(rel_pos)
                delta_t = -s if s <= 0.0 else np.linalg.norm(abs_pos - last_abs_pos) / s
                curr_t = curr_t + delta_t
                yield self._elem(curr_t, abs_pos)
                curr_pos = rel_pos
                last_abs_pos = abs_pos

    def generate(self, func):
        return np.array([p for p in self._gen(func)])

    @staticmethod
    def pause(sec):
        yield -sec, (0., 0., 0.)

    def _line_by(self, p):
        return self._speed, (p[0], p[1], p[2])

    def line_by(self, p):
        yield self._line_by(p)

    def lines_by(self, ps):
        for p in ps:
            yield self._line_by(p)


class WaypointGenerator:

    @staticmethod
    def line_y(figure: Figure):
        yield figure.pause(4)
        yield figure.line_by([0., 0.5, 0.])
        yield figure.line_by([0., -1., 0.])
        yield figure.line_by([0., 0.5, 0.])

    @staticmethod
    def stationary(figure: Figure):
        yield figure.pause(20)

    @staticmethod
    def generate(style, speed=0.15, origin=(0., 0., 0.), scale=1.0):
        assert style in WaypointGenerator.__dict__ and isinstance(WaypointGenerator.__dict__[style], staticmethod)
        return Figure(speed, origin, scale).generate(WaypointGenerator.__dict__[style].__func__)


def main(args=None):
    rclpy.init(args=args)

    flyer = TrajectoryVelocityFlyer()
    wp = WaypointGenerator.generate('line_y', origin=[-1.3, 0.0, 1.2])
    flyer.set_waypoints(wp, stabilize_sec=3.0, repeat=True)

    node = FlockSimplePath(flyer)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C detected, shutting down")
    finally:
        flyer.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
