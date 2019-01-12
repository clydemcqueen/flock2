#include "flock_base.hpp"

namespace flock_base {



FlockBase::FlockBase() : Node{"flock_base"}
{
  // TODO trim

  auto joy_cb = std::bind(&FlockBase::joy_callback, this, std::placeholders::_1);
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("joy", joy_cb);
}

void FlockBase::joy_callback(sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->buttons[joy_button_takeoff_]) {
    drone_.start_action(Action::takeoff);
  } else if (msg->buttons[joy_button_land_]) {
    drone_.start_action(Action::land);
  } else if (msg->buttons[joy_button_start_mission_]) {
    drone_.start_action(Action::start_mission);
  } else if (msg->buttons[joy_button_stop_mission_]) {
    drone_.start_action(Action::stop_mission);
  }

  // TODO trim

  drone_.twist_.linear.x = msg->axes[joy_axis_throttle_];
  drone_.twist_.linear.y = msg->axes[joy_axis_strafe_];
  drone_.twist_.linear.z = msg->axes[joy_axis_vertical_];
  drone_.twist_.angular.z = msg->axes[joy_axis_yaw_];
}

void FlockBase::spin_once()
{
  drone_.action_mgr_->spin_once();

  // If we're flying manually and the drone isn't busy, send a cmd_vel message
  if (drone_.state_ == State::fly_manual && !drone_.action_mgr_->busy()) {
    drone_.cmd_vel_pub_->publish(drone_.twist_);
  }
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
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

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

  // Spin until rclcpp::ok() returns false
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}