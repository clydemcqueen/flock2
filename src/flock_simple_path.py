#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

import numpy as np


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


class Event(list):
    """Event subscription.

    A list of callable objects. Calling an instance of this will cause a
    call to each item in the list in ascending order by index.

    Example Usage:
    >>> def f(x):
    ...     print 'f(%s)' % x
    >>> def g(x):
    ...     print 'g(%s)' % x
    >>> e = Event()
    >>> e()
    >>> e.append(f)
    >>> e(123)
    f(123)
    >>> e.remove(f)
    >>> e()
    >>> e += (f, g)
    >>> e(10)
    f(10)
    g(10)
    >>> del e[0]
    >>> e(2)
    g(2)

    """
    def __call__(self, *args, **kwargs):
        for f in self:
            f(*args, **kwargs)

    def __repr__(self):
        return "Event(%s)" % list.__repr__(self)


class TrajectoryVelocityFlyer:

    def __init__(self):
        self.controller = VelocityController()
        self.trajectory = TrajectoryHandler()
        self.

        # self._target_position = np.array([0.0, 0.0, 0.0])  # the target pos in [N, E, D]
        # self._target_velocity = np.array([0.0, 0.0, 0.0])  # the target vel in [Vn, Ve, Vd]
        # self._in_mission = True
        #
        # # initial state
        # self._flight_state = States.MANUAL
        #
        # # register all your callbacks here
        # self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        # self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        # self.register_callback(MsgID.STATE, self.state_callback)
        #
        # # get the controller that will be used
        # self._outer_controller = OuterLoopController()
        #
        # # get the trajectory handler
        # self._traj_handler = TrajectoryHandler("line_traj.txt")
        # self._start_time = 0.0  # this is the time that the flight started -> will be set on takeoff

    def start(self):
        pass

    def stop(self):
        pass

    def set_trajectory(self, waypoints):
        pass

    def cmd_subscription(self, cmd_callback):
        self.cmd_callback = cmd_callback

    # def local_position_callback(self):
    #     if self._flight_state == States.TAKEOFF:
    #         if -1.0 * self.local_position[2] > 0.95 * self._target_position[2]:
    #             self._start_time = time.time()
    #             self.waypoint_transition()
    #
    #         # TODO: decide if want to also run the position controller from here
    #         # TODO: if desired, could simply call the takeoff method, but would be more complete to also write it here
    #
    #         # NOTE: one option for testing out the controller that people could do is use the takeoff method
    #         # and then simply send the thrust command for hover, this help tune the mass of the drone - if a scale isn't accessible
    #         # NOTE: this would be an effort to just try and tie in the content from the C++ project
    #
    #         # TODO: the same dilema is of note for landing
    #         # could just open it up as a "hey if you want to see what your controller does for your entire flight"
    #
    #     elif self._flight_state == States.WAYPOINT:
    #
    #         # get the current in flight time
    #         rel_time = time.time() - self._start_time
    #
    #         # check if trajectory is completed
    #         if self._traj_handler.is_trajectory_completed(rel_time):
    #             self.landing_transition()
    #             return
    #
    #         # set the target position and velocity from the trajectory
    #         self._target_position, self._target_velocity = self._traj_handler.get_next_point(rel_time)
    #
    #         # run the outer loop controller (position controller -> to velocity command)
    #         vel_cmd = self.run_outer_controller()
    #
    #         # send the velocity command to the drone
    #         self.cmd_velocity(vel_cmd[0], vel_cmd[1], vel_cmd[2], 0.0)

    # def velocity_callback(self):
    #     if self._flight_state == States.LANDING:
    #         if abs(self.local_velocity[2]) < 0.05:
    #             self.disarming_transition()

    # def state_callback(self):
    #     if self._in_mission:
    #         if self._flight_state == States.MANUAL:
    #             self.arming_transition()
    #         elif self._flight_state == States.ARMING:
    #             if self.armed:
    #                 self.takeoff_transition()
    #         elif self._flight_state == States.DISARMING:
    #             if ~self.armed & ~self.guided:
    #                 self.manual_transition()

    def run_outer_controller(self):
        """helper function to run the outer loop controller.

        calls the outer loop controller to run the lateral position and altitude controllers to
        get the velocity vector to command.

        Returns:
            the velocity vector to command as [vn, ve, vd] in [m/s]
            numpy array of floats
        """

        lateral_vel_cmd = self.controller.lateral_position_control(self._target_position,
                                                                          self.local_position,
                                                                          self._target_velocity)
        hdot_cmd = self.controller.altitude_control(-self._target_position[2],
                                                           -self.local_position[2],
                                                           -self._target_velocity[2])

        return np.array([lateral_vel_cmd[0], lateral_vel_cmd[1], -hdot_cmd])

    # def arming_transition(self):
    #     print("arming transition")
    #     self.take_control()
    #     self.arm()
    #     self.set_home_as_current_position()
    #     self._flight_state = States.ARMING
    #
    # def takeoff_transition(self):
    #     print("takeoff transition")
    #     target_altitude = TAKEOFF_ALTITUDE
    #     self._target_position[2] = target_altitude
    #
    #     # NOTE: the configuration let's the crazyflie handle the takeoff
    #     self.takeoff(target_altitude)
    #     self._flight_state = States.TAKEOFF
    #
    # def waypoint_transition(self):
    #     print("waypoint transition")
    #     self._flight_state = States.WAYPOINT
    #
    # def landing_transition(self):
    #     print("landing transition")
    #     # NOTE: the configuration let's the crazyflie handle the takeoff
    #     self.land()
    #     self._flight_state = States.LANDING
    #
    # def disarming_transition(self):
    #     print("disarm transition")
    #     self.disarm()
    #     self.release_control()
    #     self._flight_state = States.DISARMING
    #
    # def manual_transition(self):
    #     print("manual transition")
    #     self.stop()
    #     self._in_mission = False
    #     self._flight_state = States.MANUAL
    #
    # def start(self):
    #     self.start_log("Logs", "NavLog.txt")
    #     # self.connect()
    #
    #     print("starting connection")
    #     # self.connection.start()
    #
    #     super().start()
    #
    #     # Only required if they do threaded
    #     # while self._in_mission:
    #     #    pass
    #
    #     self.stop_log()


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
        twist = Twist()
        print(msg)

    def flyer_cmd_callback(self, cmd):
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
