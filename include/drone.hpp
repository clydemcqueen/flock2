#ifndef DRONE_H
#define DRONE_H

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tello_msgs/msg/flight_data.hpp"

#include "action_mgr.hpp"

namespace flock_base {

enum class State
{
  unknown,
  landed,
  fly_manual,
  fly_mission,
};

enum class Action
{
  takeoff,
  land,
  start_mission,
  stop_mission,
  connect,
  disconnect,
};

class FlockBase;

class Drone
{
  // ROS node, for logging, setting up pubs and subs, etc.
  FlockBase *node_;

  // ROS topic namespace for this drone
  std::string ns_;

  // Flight state
  State state_ = State::unknown;

  // Drone action manager
  std::unique_ptr<ActionMgr> action_mgr_;

  // Target velocity
  geometry_msgs::msg::Twist twist_;

  // Message timestamps
  rclcpp::Time prev_flight_data_stamp_;
  rclcpp::Time prev_odom_stamp_;

  // Publications
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr start_mission_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_mission_pub_;
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

  // Start an action. Can be called from a ROS callback: fast, doesn't block.
  void start_action(Action action);

  void set_velocity(double throttle, double strafe, double vertical, double yaw);

  void spin_once();

private:

  // Transition to a new state.
  void transition_state(Action action);

  void tello_response_callback(tello_msgs::msg::TelloResponse::SharedPtr msg);
  void flight_data_callback(tello_msgs::msg::FlightData::SharedPtr msg);
  void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
};

} // namespace flock_base

#endif // DRONE_H
