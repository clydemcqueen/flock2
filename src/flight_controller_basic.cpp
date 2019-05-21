
#include "flight_controller_basic.hpp"

namespace drone_base
{

//=============================================================================
// FlightControllerBasic
//=============================================================================

  FlightControllerBasic::FlightControllerBasic(rclcpp::Node &node,
                                               rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr &cmd_vel_pub) :
    FlightControllerInterface(node, cmd_vel_pub)
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER(node_, (*this), n, t, d)

    CXT_MACRO_INIT_PARAMETERS(BASIC_CONTROLLER_ALL_PARAMS, validate_parameters);

    _reset();
  }

  void FlightControllerBasic::validate_parameters()
  {
    stabilize_time_ = rclcpp::Duration(static_cast<int64_t>(RCL_S_TO_NS(stabilize_time_sec_)));
  }

  void FlightControllerBasic::_parameters_changed(const std::vector<rclcpp::Parameter> &parameters)
  {
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED((*this), n, t)

    CXT_MACRO_PARAMETERS_CHANGED_BODY(BASIC_CONTROLLER_ALL_PARAMS, parameters, validate_parameters)
  }

  void FlightControllerBasic::_reset()
  {
    last_odom_time_ = rclcpp::Time();
  }

  void FlightControllerBasic::_set_target(int target)
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

    // Set previous target, as well as velocity
    if (target_ == 0) {
      // Takeoff case
      prev_target_ = curr_target_;
      prev_target_time_ = node_.now();
      vx_ = vy_ = vz_ = vyaw_ = 0;
    } else {
      // Typical case
      prev_target_.fromMsg(plan_.poses[target_ - 1].pose);
      prev_target_time_ = rclcpp::Time(plan_.poses[target_ - 1].header.stamp);

      auto flight_time = (curr_target_time_ - prev_target_time_).seconds();
      assert(flight_time > 0);

      // Velocity vector from previous target to this target
      vx_ = (curr_target_.x - prev_target_.x) / flight_time;
      vy_ = (curr_target_.y - prev_target_.y) / flight_time;
      vz_ = (curr_target_.z - prev_target_.z) / flight_time;
      vyaw_ = PoseUtil::norm_angle(curr_target_.yaw - prev_target_.yaw) / flight_time;

      RCLCPP_INFO(node_.get_logger(), "target %d velocity: (%g, %g, %g), yaw %g", target_, vx_, vy_, vz_, vyaw_);
    }

    // Initialize PID controllers to previous target, these will be updated in the odom callback
    x_controller_.set_target(prev_target_.x);
    y_controller_.set_target(prev_target_.y);
    z_controller_.set_target(prev_target_.z);
    yaw_controller_.set_target(prev_target_.yaw);
  }

  bool FlightControllerBasic::_odom_callback(const nav_msgs::msg::Odometry::SharedPtr &msg)
  {
    bool retVal = false;

    rclcpp::Time msg_time(msg->header.stamp);

    if (PoseUtil::is_valid_time(last_odom_time_)) {
      if (msg_time > curr_target_time_ + stabilize_time_) {
        if (curr_target_.close_enough(last_pose_)) {
          // Advance to the next target
          set_target(target_ + 1);
        } else {
          // Timeout
          retVal = true;
        }
      } else {
        // Compute expected position and set PID targets
        // The odom pipeline has a lag, so ignore messages that are older than prev_target_time_
        if (msg_time < curr_target_time_ && msg_time > prev_target_time_) {
          auto elapsed_time = (msg_time - prev_target_time_).seconds();
          x_controller_.set_target(prev_target_.x + vx_ * elapsed_time);
          y_controller_.set_target(prev_target_.y + vy_ * elapsed_time);
          z_controller_.set_target(prev_target_.z + vz_ * elapsed_time);
          yaw_controller_.set_target(PoseUtil::norm_angle(prev_target_.yaw + vyaw_ * elapsed_time));
        }

        // Compute velocity
        auto dt = (msg_time - last_odom_time_).seconds();
        double ubar_x = x_controller_.calc(last_pose_.x, dt, 0);
        double ubar_y = y_controller_.calc(last_pose_.y, dt, 0);
        double ubar_z = z_controller_.calc(last_pose_.z, dt, 0);
        double ubar_yaw = yaw_controller_.calc(last_pose_.yaw, dt, 0);

        // Rotate ubar_x and ubar_y into the body frame
        double throttle, strafe;
        PoseUtil::rotate_frame(ubar_x, ubar_y, last_pose_.yaw, throttle, strafe);

//      RCLCPP_INFO(node_.get_logger(), "%12ld "
//                  "x controller: target %7.3f, curr %7.3f, throttle %7.3f "
//                  "y controller: target %7.3f, curr %7.3f, strafe %7.3f",
//                  RCL_NS_TO_MS(msg_time.nanoseconds()),
//                  x_controller_.target(), last_pose_.x, throttle,
//                  y_controller_.target(), last_pose_.y, strafe);

        // Publish velocity
        publish_velocity(throttle, strafe, ubar_z, ubar_yaw);
      }
    }

    last_odom_time_ = msg_time;
    last_pose_.fromMsg(msg->pose.pose);
    return retVal;
  }


}