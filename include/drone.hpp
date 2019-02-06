#ifndef DRONE_H
#define DRONE_H

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tello_msgs/msg/flight_data.hpp"

#include "action_mgr.hpp"

namespace flock_base {

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

class FlockBase;

class Drone
{
  // ROS node, for logging, setting up pubs and subs, etc.
  FlockBase *node_;

  // ROS topic namespace for this drone
  std::string ns_;

  // Drone state
  rclcpp::Time prev_flight_data_stamp_;
  rclcpp::Time prev_odom_stamp_;
  State state_ = State::unknown;

  // Drone action manager
  std::unique_ptr<ActionMgr> action_mgr_;

  // Target velocity
  geometry_msgs::msg::Twist twist_;

  // Publications
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Subscriptions
  rclcpp::Subscription<tello_msgs::msg::TelloResponse>::SharedPtr tello_response_sub_;
  rclcpp::Subscription<tello_msgs::msg::FlightData>::SharedPtr flight_data_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

public:

  explicit Drone(FlockBase *node, std::string ns);
  explicit Drone(FlockBase *node) : Drone(node, "") {}
  ~Drone() {}

  std::string ns() { return ns_; }
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

  void tello_response_callback(tello_msgs::msg::TelloResponse::SharedPtr msg);
  void flight_data_callback(tello_msgs::msg::FlightData::SharedPtr msg);
  void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
};

} // namespace flock_base

#endif // DRONE_H
