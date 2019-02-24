#ifndef SIMPLE_PLANNER_H
#define SIMPLE_PLANNER_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/empty.hpp"

namespace simple_planner {

class SimplePlanner
{
  std::vector<nav_msgs::msg::Path> plans_;

public:

  explicit SimplePlanner(const std::vector<geometry_msgs::msg::PoseStamped> &landing_poses);
  ~SimplePlanner() {}

  const std::vector<nav_msgs::msg::Path> &plans() const { return plans_; }
};

} // namespace simple_planner

#endif // SIMPLE_PLANNER_H
