#ifndef FLIGHT_CONTROLLER_INTERFACE_HPP
#define FLIGHT_CONTROLLER_INTERFACE_HPP

#include "drone_pose.hpp"

namespace drone_base
{
  class FlightControllerInterface
  {
  protected:
    int target_{};                            // Current target (index into plan_)
    nav_msgs::msg::Path plan_{};              // The flight plan

    rclcpp::Node &node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr &cmd_vel_pub_;

    // Implemented by the overriding class.
    virtual void _reset() = 0;

    virtual void _set_target(int target) = 0;

    virtual bool _odom_callback(const nav_msgs::msg::Odometry::SharedPtr &msg) = 0;

    virtual void _parameters_changed(const std::vector<rclcpp::Parameter> &parameters) = 0;

  public:
    explicit FlightControllerInterface(rclcpp::Node &node,
                                       rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr &cmd_vel_pub)
      : node_{node}, cmd_vel_pub_{cmd_vel_pub}
    {}

    FlightControllerInterface() = delete;

    virtual ~FlightControllerInterface() = default;

    void reset()
    {
      plan_ = nav_msgs::msg::Path();
      _reset();
    }

    void set_target(int target)
    {
      _set_target(target);
    }

    void set_plan(const nav_msgs::msg::Path::SharedPtr &msg)
    {
      _reset();
      plan_ = *msg;
      _set_target(0);
    }

    bool odom_callback(const nav_msgs::msg::Odometry::SharedPtr &msg)
    {
      return _odom_callback(msg);
    }

    void parameters_changed(const std::vector<rclcpp::Parameter> &parameters)
    {
      _parameters_changed(parameters);
    }

    void publish_velocity(double throttle, double strafe, double vertical, double yaw)
    {
      geometry_msgs::msg::Twist twist;
      twist.linear.x = PoseUtil::clamp(throttle, -1.0, 1.0);
      twist.linear.y = PoseUtil::clamp(strafe, -1.0, 1.0);
      twist.linear.z = PoseUtil::clamp(vertical, -1.0, 1.0);
      twist.angular.z = PoseUtil::clamp(yaw, -1.0, 1.0);
      cmd_vel_pub_->publish(twist);
    }

    bool is_plan_complete()
    {
      return target_ >= plan_.poses.size();
    }

    bool have_plan()
    {
      return !plan_.poses.empty();
    }
  };
}

#endif //FLIGHT_CONTROLLER_INTERFACE_HPP
