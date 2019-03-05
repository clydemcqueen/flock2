#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/empty.hpp"

namespace local_planner {

class LocalPlannerNode : public rclcpp::Node
{
  // Global state
  bool mission_ = false;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_mission_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub_;

  // Publications
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;

public:

  explicit LocalPlannerNode();
  ~LocalPlannerNode() {}

  void spin_once();

private:

  void start_mission_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void stop_mission_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void global_plan_callback(const nav_msgs::msg::Path::SharedPtr msg);
};

} // namespace local_planner

#endif // LOCAL_PLANNER_H
