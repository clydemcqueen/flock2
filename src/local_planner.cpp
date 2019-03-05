#include "local_planner.hpp"

namespace local_planner {

LocalPlannerNode::LocalPlannerNode() : Node{"local_planner"}
{
  auto start_mission_cb = std::bind(&LocalPlannerNode::start_mission_callback, this, std::placeholders::_1);
  auto stop_mission_cb = std::bind(&LocalPlannerNode::stop_mission_callback, this, std::placeholders::_1);
  auto global_plan_cb = std::bind(&LocalPlannerNode::global_plan_callback, this, std::placeholders::_1);

  start_mission_sub_ = create_subscription<std_msgs::msg::Empty>("/start_mission", start_mission_cb);
  stop_mission_sub_ = create_subscription<std_msgs::msg::Empty>("/stop_mission", stop_mission_cb);
  global_plan_sub_ = create_subscription<nav_msgs::msg::Path>("global_plan", global_plan_cb);

  local_plan_pub_ = create_publisher<nav_msgs::msg::Path>("local_plan", 1);
}

void LocalPlannerNode::start_mission_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  mission_ = true;
}

void LocalPlannerNode::stop_mission_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  mission_ = false;
}

void LocalPlannerNode::global_plan_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
}

void LocalPlannerNode::spin_once()
{
}

} // namespace local_planner

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<local_planner::LocalPlannerNode>();
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

  rclcpp::Rate r(20);
  while (rclcpp::ok())
  {
    // Do our work
    node->spin_once();

    // Respond to incoming messages
    rclcpp::spin_some(node);

    // Wait
    r.sleep();
  }

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}