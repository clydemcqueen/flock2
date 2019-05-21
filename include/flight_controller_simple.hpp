#ifndef FLIGHT_CONTROLLER_SIMPLE_HPP
#define FLIGHT_CONTROLLER_SIMPLE_HPP

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "flight_controller_interface.hpp"
#include "pid.hpp"

namespace drone_base
{
  class FlightControllerSimpleContext;

  class FlightControllerSimple : public FlightControllerInterface
  {
    std::shared_ptr<FlightControllerSimpleContext> cxt_;

    DronePose last_pose_;                   // Last pose from odom
    rclcpp::Time last_odom_time_;           // Last pose from odom

    DronePose prev_target_;                 // Previous target pose
    DronePose curr_target_;                 // Current target pose
    rclcpp::Time prev_target_time_;         // Time we left the previous target
    rclcpp::Time curr_target_time_;         // Deadline to hit the current target
    double vx_{}, vy_{}, vz_{}, vyaw_{};    // Velocity required to hit the current target

    // PID controllers
    pid::Controller x_controller_{false, 0.1, 0, 0};
    pid::Controller y_controller_{false, 0.1, 0, 0};
    pid::Controller z_controller_{false, 0.1, 0, 0};
    pid::Controller yaw_controller_{true, 0.2, 0, 0};

    void validate_parameters();

  public:
    FlightControllerSimple(rclcpp::Node &node, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr &cmd_vel_pub);

    void _reset() override;

    void _set_target(int target) override;

    bool _odom_callback(const nav_msgs::msg::Odometry::SharedPtr &msg) override;

    void _parameters_changed(const std::vector<rclcpp::Parameter> &parameters) override;
  };
}
#endif //FLIGHT_CONTROLLER_SIMPLE_HPP
