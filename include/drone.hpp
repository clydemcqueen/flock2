#ifndef DRONE_H
#define DRONE_H

#include "geometry_msgs/msg/twist.hpp"
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
public:  // TODO

  // Node
  FlockBase *node_;

  // Actions
  std::unique_ptr<ActionMgr> action_mgr_;
  geometry_msgs::msg::Twist twist_;

  // Flight state
  State state_ = State::unknown;

  // Publications
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr start_mission_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_mission_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Subscriptions
  rclcpp::Subscription<tello_msgs::msg::TelloResponse>::SharedPtr tello_response_sub_;
  rclcpp::Subscription<tello_msgs::msg::FlightData>::SharedPtr flight_data_sub_;

  explicit Drone(FlockBase *node);
  ~Drone() {}

  // Start an action. Can be called from a ROS callback: fast, doesn't block.
  void start_action(Action action);

  // Transition to a new state.
  void transition_state(Action action);

  void tello_response_callback(tello_msgs::msg::TelloResponse::SharedPtr msg);
  void flight_data_callback(tello_msgs::msg::FlightData::SharedPtr msg);
};

} // namespace flock_base

#endif // DRONE_H
