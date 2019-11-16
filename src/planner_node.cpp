#include "planner_node.hpp"

#include "simple_planner.hpp"

namespace planner_node
{

//====================
// Constants
//====================

// Spin rate
  const int SPIN_RATE = 20;

// Arena constraints
  const double MIN_ARENA_Z = 1.5;
  const double MIN_ARENA_XY = 2.0;
  const double GROUND_EPSILON = 1.2;

//====================
// Utilities
//====================

// Are we on the ground?
  inline bool on_the_ground(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    return std::abs(msg->pose.pose.position.z) < GROUND_EPSILON;
  }

//====================
// DroneInfo
//====================

  DroneInfo::DroneInfo(rclcpp::Node *node, std::string ns) : ns_{ns}, valid_landing_pose_{false}
  {
    auto odom_cb = std::bind(&DroneInfo::odom_callback, this, std::placeholders::_1);

    // TODO move topics to cxt
    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(ns + "/base_odom", 10, odom_cb);
    plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(ns + "/plan", 1);
  }

  void DroneInfo::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!valid_landing_pose_ && on_the_ground(msg)) {
      landing_pose_.header = msg->header;
      landing_pose_.pose = msg->pose.pose;
      landing_pose_.pose.position.z = 0;
      valid_landing_pose_ = true;
    }
  }

//====================
// PlannerNode
//====================

  PlannerNode::PlannerNode() : Node{"planner_node"}
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(PLANNER_NODE_ALL_PARAMS, validate_parameters)

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), PLANNER_NODE_ALL_PARAMS, validate_parameters)

    // Get drone namespaces
    if (cxt_.drones_.size() > 1) {
      RCLCPP_INFO(get_logger(), "%d drones", cxt_.drones_.size());
    } else {
      RCLCPP_INFO(get_logger(), "1 drone");
    }

    // Get arena
    RCLCPP_INFO(get_logger(), "arena is (0, 0, 0) to (%g, %g, %g)", cxt_.arena_x_, cxt_.arena_y_, cxt_.arena_z_);

    auto start_mission_cb = std::bind(&PlannerNode::start_mission_callback, this, std::placeholders::_1);
    auto stop_mission_cb = std::bind(&PlannerNode::stop_mission_callback, this, std::placeholders::_1);

    start_mission_sub_ = create_subscription<std_msgs::msg::Empty>("/start_mission", 10, start_mission_cb);
    stop_mission_sub_ = create_subscription<std_msgs::msg::Empty>("/stop_mission", 10, stop_mission_cb);

    for (auto i = cxt_.drones_.begin(); i != cxt_.drones_.end(); i++) {
      drones_.push_back(std::make_shared<DroneInfo>(this, *i));
    }
  }

  void PlannerNode::create_and_publish_plans()
  {
    // Check arena volume
    if (std::abs(cxt_.arena_x_) < MIN_ARENA_XY || std::abs(cxt_.arena_y_) < MIN_ARENA_XY ||
        cxt_.arena_z_ < MIN_ARENA_Z) {
      RCLCPP_ERROR(get_logger(), "arena must be at least (%g, %g, %g)", MIN_ARENA_XY, MIN_ARENA_XY, MIN_ARENA_Z);
      return;
    }

    // Make sure we have landing poses
    for (auto i = drones_.begin(); i != drones_.end(); i++) {
      if (!(*i)->valid_landing_pose()) {
        RCLCPP_ERROR(get_logger(), "waiting for landing pose for %s (and possibly more)", (*i)->ns().c_str());
        return;
      }
    }

    // Create N plans
    std::vector<geometry_msgs::msg::PoseStamped> landing_poses;
    for (auto i = drones_.begin(); i != drones_.end(); i++) {
      landing_poses.push_back((*i)->landing_pose());
    }
    simple_planner::SimplePlanner planner(landing_poses);
    std::vector<nav_msgs::msg::Path> plans = planner.plans(now());
    RCLCPP_INFO(get_logger(), "plan(s) created");

    // Publish N plans
    for (int i = 0; i < drones_.size(); i++) {
      drones_[i]->plan_pub()->publish(plans[i]);
    }
  }

  void PlannerNode::start_mission_callback(const std_msgs::msg::Empty::SharedPtr msg)
  {
    (void) msg;
    RCLCPP_INFO(get_logger(), "start mission");
    create_and_publish_plans();
  }

  void PlannerNode::stop_mission_callback(const std_msgs::msg::Empty::SharedPtr msg)
  {
    (void) msg;
    RCLCPP_INFO(get_logger(), "stop mission");
  }

  void PlannerNode::validate_parameters()
  {
    RCLCPP_INFO(get_logger(), "PlannerNode Parameters");

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    PLANNER_NODE_ALL_PARAMS
  }

} // namespace planner_node

//====================
// main
//====================

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<planner_node::PlannerNode>();
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

  // Spin until rclcpp::ok() returns false
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}