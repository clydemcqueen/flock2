
#include "flight_controller_simple.hpp"

namespace drone_base
{
  FlightControllerSimple::FlightControllerSimple(rclcpp::Node &node,
                                                 rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr &cmd_vel_pub) :
    FlightControllerInterface(node, cmd_vel_pub)
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER(node_, (*this), n, t, d)

    CXT_MACRO_INIT_PARAMETERS(SIMPLE_CONTROLLER_ALL_PARAMS, validate_parameters);

    _reset();
  }

  void FlightControllerSimple::validate_parameters()
  {
    stabilize_time_ = rclcpp::Duration(static_cast<int64_t>(RCL_S_TO_NS(stabilize_time_sec_)));
    x_controller_.set_coefficients(pid_x_kp_, pid_x_ki_, pid_x_kd_);
    y_controller_.set_coefficients(pid_y_kp_, pid_y_ki_, pid_y_kd_);
    z_controller_.set_coefficients(pid_z_kp_, pid_z_ki_, pid_z_kd_);
    yaw_controller_.set_coefficients(pid_yaw_kp_, pid_yaw_ki_, pid_yaw_kd_);
  }

  void FlightControllerSimple::_parameters_changed(const std::vector<rclcpp::Parameter> &parameters)
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED((*this), n, t)

    CXT_MACRO_PARAMETERS_CHANGED_BODY(SIMPLE_CONTROLLER_ALL_PARAMS, parameters, validate_parameters)
  }

  void FlightControllerSimple::_reset()
  {
    last_odom_time_ = rclcpp::Time();
  }

  void FlightControllerSimple::_set_target(int target)
  {
    target_ = target;

    // Handle "done" case
    if (target_ < 0 || target_ >= plan_.poses.size()) {
      return;
    }

    // Set current target
    curr_target_.fromMsg(plan_.poses[target_].pose);
    curr_target_time_ = rclcpp::Time(plan_.poses[target_].header.stamp) - stabilize_time_;

    RCLCPP_INFO(node_.get_logger(), "target %d position: (%g, %g, %g), yaw %g, time %12ld",
                target_,
                curr_target_.x,
                curr_target_.y,
                curr_target_.z,
                curr_target_.yaw,
                RCL_NS_TO_MS(curr_target_time_.nanoseconds()));

    // Initialize PID controllers to previous target, these will be updated in the odom callback
    x_controller_.set_target(curr_target_.x);
    y_controller_.set_target(curr_target_.y);
    z_controller_.set_target(curr_target_.z);
    yaw_controller_.set_target(curr_target_.yaw);
  }

  bool FlightControllerSimple::_odom_callback(const nav_msgs::msg::Odometry::SharedPtr &msg)
  {
    bool retVal = false;

    rclcpp::Time msg_time(msg->header.stamp);
    DronePose pose;
    pose.fromMsg(msg->pose.pose);

    if (PoseUtil::is_valid_time(last_odom_time_)) {
      if (msg_time > curr_target_time_ + stabilize_time_) {
        if (curr_target_.close_enough(pose, close_enough_xyz_, close_enough_yaw_)) {
          // Advance to the next target
          set_target(target_ + 1);
        } else {
          // Timeout
          retVal = true;
        }
      } else {

        // Compute the velocity needed to get to the target.
        auto dt = (msg_time - last_odom_time_).seconds();
        double ubar_x = x_controller_.calc(pose.x, dt, 0);
        double ubar_y = y_controller_.calc(pose.y, dt, 0);
        double ubar_z = z_controller_.calc(pose.z, dt, 0);
        double ubar_yaw = yaw_controller_.calc(pose.yaw, dt, 0);

        // Rotate ubar_x and ubar_y into the body frame
        double throttle, strafe;
        PoseUtil::rotate_frame(ubar_x, ubar_y, pose.yaw, throttle, strafe);

//        RCLCPP_INFO(node_.get_logger(), "%12ld "
//                                        "x controller: target %7.3f, curr %7.3f, throttle %7.3f "
//                                        "y controller: target %7.3f, curr %7.3f, strafe %7.3f",
//                    RCL_NS_TO_MS(msg_time.nanoseconds()),
//                    x_controller_.target(), last_pose_.x, throttle,
//                    y_controller_.target(), last_pose_.y, strafe);

        // Publish velocity
        publish_velocity(throttle, strafe, ubar_z, ubar_yaw);
      }
    }

    last_odom_time_ = msg_time;
    return retVal;
  }
}
