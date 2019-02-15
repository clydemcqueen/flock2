#ifndef DRONE_BASE_H
#define DRONE_BASE_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tello_msgs/msg/flight_data.hpp"

#include "joystick.hpp"
#include "action_mgr.hpp"

namespace drone_base {

// Flight states
enum class State
{
  unknown,        // No flight data
  ready,          // Ready for manual flight
  flight,         // Flying (autonomous operation not available)
  ready_odom,     // Ready for manual or autonomous flight
  flight_odom,    // Flying (autonomous operation available)
  low_battery,    // Low battery (must be swapped)
};

// Events happen
enum class Event
{
  connected,        // Connected to the drone
  disconnected,     // Lost the connection
  odometry_started, // Odometry started
  odometry_stopped, // Odometry stopped
  low_battery,      // Low battery detected
};

// The user or autonomous controller can take actions
enum class Action
{
  takeoff,
  land,
};

class DroneBase : public rclcpp::Node
{
  // Drone state
  bool mission_ = false;
  rclcpp::Time prev_flight_data_stamp_;
  rclcpp::Time prev_odom_stamp_;
  State state_ = State::unknown;

  // Drone action manager
  std::unique_ptr<ActionMgr> action_mgr_;

  // Target velocity
  geometry_msgs::msg::Twist twist_;

  // Joystick assignments
  int joy_axis_throttle_ = JOY_AXIS_RIGHT_FB;
  int joy_axis_strafe_ = JOY_AXIS_RIGHT_LR;
  int joy_axis_vertical_ = JOY_AXIS_LEFT_FB;
  int joy_axis_yaw_ = JOY_AXIS_LEFT_LR;
  int joy_button_takeoff_ = JOY_BUTTON_MENU;
  int joy_button_land_ = JOY_BUTTON_VIEW;
  int joy_button_shift_ = JOY_BUTTON_LEFT_BUMPER;
  int joy_axis_trim_lr_ = JOY_AXIS_TRIM_LR;
  int joy_axis_trim_fb_ = JOY_AXIS_TRIM_FB;

  // Publications
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_mission_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<tello_msgs::msg::TelloResponse>::SharedPtr tello_response_sub_;
  rclcpp::Subscription<tello_msgs::msg::FlightData>::SharedPtr flight_data_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

public:

  explicit DroneBase();
  ~DroneBase() {}

  bool receiving_flight_data() { return prev_flight_data_stamp_.nanoseconds() > 0; }
  bool receiving_odometry() { return prev_odom_stamp_.nanoseconds() > 0; }

  // Start an action. Can be called from a ROS callback: fast, doesn't block.
  void start_action(Action action);

  void set_velocity(double throttle, double strafe, double vertical, double yaw);

  void spin_once();

private:

  // Transition to a new state.
  void transition_state(Action action);
  void transition_state(Event event);
  void transition_state(State next_state);

  void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg);
  void start_mission_callback(std_msgs::msg::Empty::SharedPtr msg);
  void stop_mission_callback(std_msgs::msg::Empty::SharedPtr msg);
  void tello_response_callback(tello_msgs::msg::TelloResponse::SharedPtr msg);
  void flight_data_callback(tello_msgs::msg::FlightData::SharedPtr msg);
  void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
};

} // namespace drone_base

#endif // DRONE_BASE_H
