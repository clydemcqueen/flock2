#ifndef PLANNER_H
#define PLANNER_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/empty.hpp"

#include "ros2_shared/context_macros.hpp"

namespace planner_node {

//=============================================================================
// DroneInfo
//=============================================================================

  class DroneInfo
{
  std::string ns_;

  // Pose for takeoff and landing
  bool valid_landing_pose_;
  geometry_msgs::msg::PoseStamped landing_pose_;

  // At the moment, odometry is only used to capture the landing pad location
  // In the future the plan might be updated based on current drone locations
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publish a plan at 1Hz
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

public:

  explicit DroneInfo(rclcpp::Node *node, std::string ns);
  ~DroneInfo() {};

  std::string ns() const { return ns_; }
  bool valid_landing_pose() const { return valid_landing_pose_; }
  const geometry_msgs::msg::PoseStamped &landing_pose() const { return landing_pose_; }
  const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub() const { return plan_pub_; }
};

//=============================================================================
// PlannerNode parameters
//=============================================================================

#define PLANNER_NODE_ALL_PARAMS \
  CXT_MACRO_MEMBER(               /* */ \
  arena_x, \
  double, 2.0) \
  CXT_MACRO_MEMBER(               /* */ \
  arena_y, \
  double, 2.0) \
  CXT_MACRO_MEMBER(               /* */ \
  arena_z, \
  double, 2.0) \
  CXT_MACRO_MEMBER(               /*  */ \
  drones, \
  std::vector<std::string>, "solo") \
  /* End of list */

  struct PlannerNodeContext
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
    PLANNER_NODE_ALL_PARAMS
  };

//=============================================================================
// PlannerNode
//=============================================================================

class PlannerNode : public rclcpp::Node
{
  PlannerNodeContext cxt_{};

  // Per-drone info
  std::vector<std::shared_ptr<DroneInfo>> drones_;

  // Global subscriptions
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_mission_sub_;

public:

  explicit PlannerNode();
  ~PlannerNode() {}

private:

  void start_mission_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void stop_mission_callback(const std_msgs::msg::Empty::SharedPtr msg);

  void create_and_publish_plans();
  void validate_parameters();
};

} // namespace planner_node

#endif // PLANNER_H
