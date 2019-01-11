#!/usr/bin/env python

from enum import Enum

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from tello_msgs.msg import FlightData
from tello_msgs.msg import TelloResponse
from tello_msgs.srv import TelloAction

from action_mgr import ActionMgr

# XBox One joystick axes and buttons
_joy_axis_left_lr = 0           # Left stick left/right; 1.0 is left and -1.0 is right
_joy_axis_left_fb = 1           # Left stick forward/back; 1.0 is forward and -1.0 is back
_joy_axis_left_trigger = 2      # Left trigger
_joy_axis_right_lr = 3          # Right stick left/right; 1.0 is left and -1.0 is right
_joy_axis_right_fb = 4          # Right stick forward/back; 1.0 is forward and -1.0 is back
_joy_axis_right_trigger = 5     # Right trigger
_joy_axis_trim_lr = 6           # Trim left/right; 1.0 for left and -1.0 for right
_joy_axis_trim_fb = 7           # Trim forward/back; 1.0 for forward and -1.0 for back
_joy_button_A = 0               # A button
_joy_button_B = 1               # B button
_joy_button_X = 2               # X button
_joy_button_Y = 3               # Y button
_joy_button_left_bumper = 4     # Left bumper
_joy_button_right_bumper = 5    # Right bumper
_joy_button_view = 6            # View button
_joy_button_menu = 7            # Menu button
_joy_button_logo = 8            # XBox logo button
_joy_button_left_stick = 9      # Left stick button
_joy_button_right_stick = 10    # Right stick button


class Actions(Enum):
    """
    Actions are things that
        (a) cause state transitions, and/or
        (b) get sent to the drone
    """

    TAKEOFF = 'takeoff'
    LAND = 'land'
    START_MISSION = 'start_mission'
    STOP_MISSION = 'stop_mission'
    CONNECT = 'connect'
    DISCONNECT = 'disconnect'


class FlightStates(Enum):
    """
    FlightStates determine which Actions are allowed.
    """

    UNKNOWN = 0         # Just starting, lost connection, etc.
    LANDED = 1          # On the ground
    FLY_MANUAL = 2      # In the air, manual operation
    FLY_MISSION = 3     # In the air, autonomous operation


class Transition(object):
    """
    Transition from one flight state to another.
    Internal actions don't need to be sent to the drone, e.g., START_MISSION
    """

    def __init__(self, curr_state, action, next_state, internal):
        self.curr_state = curr_state    # If we're in this state...
        self.action = action            # and we apply this action...
        self.next_state = next_state    # we'll end up here.
        self.internal = internal        # True if this is an internal action


def find_transition(state: FlightStates, action: Actions):
    for transition in _transitions:
        if transition.curr_state == state and transition.action == action:
            return transition
    return None


# Allowed transitions
_transitions = [
    # Connect / disconnect
    Transition(FlightStates.UNKNOWN, Actions.CONNECT, FlightStates.LANDED, True),
    Transition(FlightStates.LANDED, Actions.DISCONNECT, FlightStates.UNKNOWN, True),
    Transition(FlightStates.FLY_MANUAL, Actions.DISCONNECT, FlightStates.UNKNOWN, True),
    Transition(FlightStates.FLY_MISSION, Actions.DISCONNECT, FlightStates.UNKNOWN, True),

    # Take off / land
    Transition(FlightStates.LANDED, Actions.TAKEOFF, FlightStates.FLY_MANUAL, False),
    Transition(FlightStates.FLY_MANUAL, Actions.LAND, FlightStates.LANDED, False),

    # Start / stop mission
    Transition(FlightStates.FLY_MANUAL, Actions.START_MISSION, FlightStates.FLY_MISSION, True),
    Transition(FlightStates.FLY_MISSION, Actions.STOP_MISSION, FlightStates.FLY_MANUAL, True),
]


class FlockBase(Node):
    """
    FlockBase manages flight states, state transitions and drone actions.
    """

    class Axes(Enum):
        THROTTLE = 0
        STRAFE = 1
        VERTICAL = 2
        YAW = 3

    def __init__(self):
        super().__init__('flock_base')
        self.get_logger().set_level(LoggingSeverity.INFO)

        self._trim_speed = 0.2

        # Actions:
        self._action_mgr: ActionMgr = None
        self._twist = Twist()

        # Flight state
        self._flight_state = FlightStates.UNKNOWN

        # Joystick assignments
        self._joy_axis_throttle = _joy_axis_right_fb
        self._joy_axis_strafe = _joy_axis_right_lr
        self._joy_axis_vertical = _joy_axis_left_fb
        self._joy_axis_yaw = _joy_axis_left_lr
        self._joy_button_takeoff = _joy_button_menu
        self._joy_button_land = _joy_button_view
        self._joy_button_shift = _joy_button_left_bumper
        self._joy_button_stop_mission = _joy_button_A
        self._joy_button_start_mission = _joy_button_B
        self._joy_axis_trim_lr = _joy_axis_trim_lr
        self._joy_axis_trim_fb = _joy_axis_trim_fb

        # Trim axis commands
        self._trim_targets_lr = {
            (-1, False): (self.Axes.STRAFE, -1.0),
            (1, False): (self.Axes.STRAFE, 1.0),
            (-1, True): (self.Axes.YAW, -1.0),
            (1, True): (self.Axes.YAW, 1.0),
        }
        self._trim_targets_fb = {
            (-1, False): (self.Axes.THROTTLE, -1.0),
            (1, False): (self.Axes.THROTTLE, 1.0),
            (-1, True): (self.Axes.VERTICAL, -1.0),
            (1, True): (self.Axes.VERTICAL, 1.0),
        }

        # Publications
        self._start_mission_pub = self.create_publisher(Empty, 'start_mission')
        self._stop_mission_pub = self.create_publisher(Empty, 'stop_mission')
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel')

        # Subscriptions
        self.create_subscription(TelloResponse, 'tello_response', self.tello_response_callback)
        self.create_subscription(FlightData, 'flight_data', self.flight_data_callback)
        self.create_subscription(Joy, 'joy', self.joy_callback)

        # Services
        self._tello_client = self.create_client(TelloAction, 'tello_action')

    def start_action(self, action: Actions):
        """
        Starts an action. Can be called from a ROS callback: fast, doesn't block.
        """
        if self._action_mgr:
            self.get_logger().debug('busy, dropping {}'.format(action))
            return

        transition = find_transition(self._flight_state, action)
        if transition is None:
            self.get_logger().debug('{} not allowed in {}'.format(action, self._flight_state))
            return

        if transition.internal:
            self.internal_action(action)
        else:
            self.get_logger().info('initiating {}'.format(action))
            self._action_mgr = ActionMgr(self.get_logger(), self._tello_client, action, action.value)

    def spin_once(self):
        """
        spin_once advances the action, and sends periodic messages.
        """
        if self._action_mgr:
            self._action_mgr.advance()

        # If we're flying manually and the drone isn't busy, send a cmd_vel message
        if self._flight_state == FlightStates.FLY_MANUAL and self._action_mgr is None:
            self._cmd_vel_pub.publish(self._twist)

    def tello_response_callback(self, msg: TelloResponse):
        """
        tello_response_callback completes the action.
        """
        if self._action_mgr:
            self._action_mgr.complete(msg)
            if self._action_mgr.state == ActionMgr.States.SUCCEEDED:
                self.transition_state(self._action_mgr.action_code)
            elif self._action_mgr.state == ActionMgr.States.FAILED_LOST_CONNECTION:
                self.transition_state(Actions.DISCONNECT)
            self._action_mgr = None

        else:
            self.get_logger().error("unexpected message on tello_response")

    def internal_action(self, action):
        if action == Actions.START_MISSION:
            self._start_mission_pub.publish((Empty()))
        elif action == Actions.STOP_MISSION:
            self._stop_mission_pub.publish(Empty())

        self.transition_state(action)

    def transition_state(self, action: Actions):
        transition = find_transition(self._flight_state, action)
        if transition is None:
            self.get_logger().error('{} not allowed in {}'.format(action, self._flight_state))
            return

        self.get_logger().info('transition to {}'.format(transition.next_state))
        self._flight_state = transition.next_state

    def flight_data_callback(self, _):
        if self._flight_state == FlightStates.UNKNOWN:
            self.get_logger().info('receiving flight data')
            self.transition_state(Actions.CONNECT)

    def joy_axis_trim_process(self, msg, axis_id, trim_targets, twist):
        axis_value = msg.axes[axis_id]
        axis_state = -1 if axis_value < -0.5 else 1 if axis_value > 0.5 else 0
        left_bumper_pressed = msg.buttons[self._joy_button_shift] != 0
        key = (axis_state, left_bumper_pressed)
        if key not in trim_targets:
            return False
        twist_field, twist_sign = trim_targets[key]
        twist_value = twist_sign * self._trim_speed
        print(key, trim_targets[key])
        if twist_field == self.Axes.THROTTLE:
            twist.linear.x = twist_value
        elif twist_field == self.Axes.STRAFE:
            twist.linear.y = twist_value
        elif twist_field == self.Axes.VERTICAL:
            twist.linear.z = twist_value
        else:
            twist.angular.z = twist_value
        return True

    def joy_callback(self, msg: Joy):

        if msg.buttons[self._joy_button_takeoff] != 0:
            self.start_action(Actions.TAKEOFF)
        elif msg.buttons[self._joy_button_land] != 0:
            self.start_action(Actions.LAND)
        elif msg.buttons[self._joy_button_start_mission]:
            self.start_action(Actions.START_MISSION)
        elif msg.buttons[self._joy_button_stop_mission]:
            self.start_action(Actions.STOP_MISSION)

        # Set self._twist if we're flying manually
        if self._flight_state == FlightStates.FLY_MANUAL:
            trim_lr_pressed = self.joy_axis_trim_process(msg, self._joy_axis_trim_lr, self._trim_targets_lr, self._twist)
            trim_fb_pressed = self.joy_axis_trim_process(msg, self._joy_axis_trim_fb, self._trim_targets_fb, self._twist)

            if not trim_lr_pressed and not trim_fb_pressed:
                self._twist.linear.x = msg.axes[self._joy_axis_throttle]   # +x is forward, -x is back
                self._twist.linear.y = msg.axes[self._joy_axis_strafe]     # +y is left, -y is right
                self._twist.linear.z = msg.axes[self._joy_axis_vertical]   # +z is ascend, -z is descend
                self._twist.angular.z = msg.axes[self._joy_axis_yaw]       # +yaw is ccw, -yaw is cw


def main(args=None):
    rclpy.init(args=args)
    node = FlockBase()

    try:
        while rclpy.ok():
            node.spin_once()
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C detected, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
