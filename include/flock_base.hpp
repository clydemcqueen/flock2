#ifndef FLOCK_BASE_H
#define FLOCK_BASE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/empty.hpp"

#include "joystick.hpp"

namespace flock_base {

class FlockBase : public rclcpp::Node
{
  // Global state
  bool mission_;

  // Drone namespaces
  std::vector<std::string> drones_;

  // Users can use the joystick to manually control one drone at a time
  int manual_control_{0};

  // Joystick assignments
  int joy_button_stop_mission_ = JOY_BUTTON_A;
  int joy_button_start_mission_ = JOY_BUTTON_B;
  int joy_button_next_drone_ = JOY_BUTTON_RIGHT_BUMPER;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  // Publications
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr> joy_pubs_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr start_mission_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_mission_pub_;

public:

  explicit FlockBase();
  ~FlockBase() {}

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  void spin_once();
};

} // namespace flock_base

#endif // FLOCK_BASE_H
