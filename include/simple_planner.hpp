#ifndef SIMPLE_PLANNER_H
#define SIMPLE_PLANNER_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/empty.hpp"

namespace simple_planner
{

  class SimplePlanner
  {
    int num_drones_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;

  public:

    explicit SimplePlanner(const std::vector<geometry_msgs::msg::PoseStamped> &landing_poses);

    ~SimplePlanner()
    {}

    std::vector<nav_msgs::msg::Path> plans(const rclcpp::Time &now) const;
  };

} // namespace simple_planner

#endif // SIMPLE_PLANNER_H
