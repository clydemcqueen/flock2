#ifndef DRONE_BASE_H
#define DRONE_BASE_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tello_msgs/msg/flight_data.hpp"

#include "action_mgr.hpp"
#include "context_macros.hpp"
#include "joystick.hpp"

namespace drone_base {

//=============================================================================
// States, events and actions
//=============================================================================

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

//=============================================================================
// DroneBase Context
//=============================================================================

#define DRONE_BASE_ALL_PARAMS \
  CXT_MACRO_MEMBER(               /* Error if no additional flight_data message within this duration */ \
  flight_data_timeout_sec, \
  double, 1.5) \
  CXT_MACRO_MEMBER(               /* Error if no additional odom message within this duration */ \
  odom_timeout_sec, \
  double, 1.5) \
  /* End of list */

#define DRONE_BASE_ALL_OTHERS \
  CXT_MACRO_MEMBER(             /* Error if no additional flight_data message within this duration */ \
  flight_data_timeout,  \
  rclcpp::Duration, 0) \
  CXT_MACRO_MEMBER(             /* Error if no additional odom message within this duration */ \
  odom_timeout,  \
  rclcpp::Duration, 0) \
  /* End of list */


#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

struct DroneBaseContext
{
  DRONE_BASE_ALL_PARAMS
  DRONE_BASE_ALL_OTHERS
};

//=============================================================================
// DroneBase node
//=============================================================================

class FlightControllerInterface;

class DroneBase : public rclcpp::Node
{
  // DroneBase context containing parameters
  DroneBaseContext cxt_;

  // Drone state
  State state_ = State::unknown;

  // Flight data
  rclcpp::Time flight_data_time_;

  // Odom data
  rclcpp::Time odom_time_;

  // Drone action manager
  std::unique_ptr<ActionMgr> action_mgr_;

  // Mission state
  bool mission_ = false;                  // We're in a mission (flying autonomously)
  std::unique_ptr<FlightControllerInterface> fc_{};

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
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;

public:

  explicit DroneBase();

  void spin_once();

private:
  void validate_parameters();
  void parameters_changed(const std::vector<rclcpp::Parameter> &parameters);

  // Callbacks
  void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg);
  void start_mission_callback(std_msgs::msg::Empty::SharedPtr msg);
  void stop_mission_callback(std_msgs::msg::Empty::SharedPtr msg);
  void tello_response_callback(tello_msgs::msg::TelloResponse::SharedPtr msg);
  void flight_data_callback(tello_msgs::msg::FlightData::SharedPtr msg);
  void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
  void plan_callback(nav_msgs::msg::Path::SharedPtr msg);

  // State transition
  void start_action(Action action);
  void transition_state(Action action);
  void transition_state(Event event);
  void transition_state(State next_state);

  // All stop: set twist_ to 0, 0, 0, 0 and publish
  void all_stop();

  // Stop mission
  void stop_mission();
};

} // namespace drone_base

#endif // DRONE_BASE_H
