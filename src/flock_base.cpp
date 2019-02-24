#include "flock_base.hpp"

namespace flock_base {

FlockBase::FlockBase() : Node{"flock_base"}
{
  // Get drone namespaces
  if (get_parameter("drones", drones_)) {
    RCLCPP_INFO(get_logger(), "%d drones, joystick controls %s, right bumper to change",
      drones_.size(), drones_[manual_control_].c_str());
  } else {
    // A single drone always has the namespace "solo"
    RCLCPP_INFO(get_logger(), "1 drone");
    drones_.push_back("solo");
  }

  auto joy_cb = std::bind(&FlockBase::joy_callback, this, std::placeholders::_1);
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("joy", joy_cb);

  start_mission_pub_ = create_publisher<std_msgs::msg::Empty>("/start_mission", 1);
  stop_mission_pub_ = create_publisher<std_msgs::msg::Empty>("/stop_mission", 1);

  // Create N joy publishers
  for (auto i = drones_.begin(); i != drones_.end(); i++) {
    joy_pubs_.push_back(create_publisher<sensor_msgs::msg::Joy>((*i) + "/joy", 1));
  }
}

inline bool button_down(const sensor_msgs::msg::Joy::SharedPtr curr, const sensor_msgs::msg::Joy &prev, int index)
{
  return curr->buttons[index] && !prev.buttons[index];
}

void FlockBase::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  static sensor_msgs::msg::Joy prev_msg;

  // Stop/start a mission
  if (mission_ && button_down(msg, prev_msg, joy_button_stop_mission_)) {
    stop_mission_pub_->publish(std_msgs::msg::Empty());
    mission_ = false;
  } else if (!mission_ && button_down(msg, prev_msg, joy_button_start_mission_)) {
    start_mission_pub_->publish(std_msgs::msg::Empty());
    mission_ = true;
  }

  // Ignore further input if we're in a mission
  if (mission_) {
    prev_msg = *msg;
    return;
  }

  // Toggle between drones
  if (button_down(msg, prev_msg, joy_button_next_drone_)) {
    if (drones_.size() < 2) {
      RCLCPP_WARN(get_logger(), "there's only 1 drone");
    } else {
      if (++manual_control_ >= drones_.size()) {
        manual_control_ = 0;
      }
      RCLCPP_INFO(get_logger(), "joystick controls %s", drones_[manual_control_].c_str());
    }
  }

  // Send joy message to the drone
  joy_pubs_[manual_control_]->publish(msg);

  prev_msg = *msg;
}

void FlockBase::spin_once()
{
}

} // namespace flock_base

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<flock_base::FlockBase>();
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