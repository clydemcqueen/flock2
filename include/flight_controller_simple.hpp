#ifndef FLIGHT_CONTROLLER_SIMPLE_HPP
#define FLIGHT_CONTROLLER_SIMPLE_HPP

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "context_macros.hpp"
#include "flight_controller_interface.hpp"
#include "pid.hpp"

namespace drone_base
{
#define SIMPLE_CONTROLLER_ALL_PARAMS \
  CXT_MACRO_MEMBER(               /* Allow drone to stabilize for this duration */ \
  stabilize_time_sec, \
  double, 5.) \
  CXT_MACRO_MEMBER(               /* PID coefficients */ \
  pid_x_kp, \
  double, 1.0) \
  CXT_MACRO_MEMBER(               /* PID coefficients */ \
  pid_x_kd, \
  double, 0.15) \
  CXT_MACRO_MEMBER(               /* PID coefficients */ \
  pid_x_ki, \
  double, 0.0) \
  CXT_MACRO_MEMBER(               /* PID coefficients */ \
  pid_y_kp, \
  double, 1.0) \
  CXT_MACRO_MEMBER(               /* PID coefficients */ \
  pid_y_kd, \
  double, 0.15) \
  CXT_MACRO_MEMBER(               /* PID coefficients */ \
  pid_y_ki, \
  double, 0.0) \
  CXT_MACRO_MEMBER(               /* PID coefficients */ \
  pid_z_kp, \
  double, 0.1) \
  CXT_MACRO_MEMBER(               /* PID coefficients */ \
  pid_z_kd, \
  double, 0.0) \
  CXT_MACRO_MEMBER(               /* PID coefficients */ \
  pid_z_ki, \
  double, 0.0) \
  CXT_MACRO_MEMBER(               /* PID coefficients */ \
  pid_yaw_kp, \
  double, 0.1) \
  CXT_MACRO_MEMBER(               /* PID coefficients */ \
  pid_yaw_kd, \
  double, 0.0) \
  CXT_MACRO_MEMBER(               /* PID coefficients */ \
  pid_yaw_ki, \
  double, 0.0) \
  CXT_MACRO_MEMBER(               /* Waypoint reached criterion */ \
  close_enough_xyz, \
  double, 0.15) \
  CXT_MACRO_MEMBER(               /* Waypoint reached criterion */ \
  close_enough_yaw, \
  double, 0.15) \
  /* End of list */

#define SIMPLE_CONTROLLER_ALL_OTHERS \
  CXT_MACRO_MEMBER(             /* Allow drone to stabilize for this duration */ \
  stabilize_time,  \
  rclcpp::Duration, 0) \
  /* End of list */

  class FlightControllerSimple : public FlightControllerInterface
  {

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
    SIMPLE_CONTROLLER_ALL_PARAMS
    SIMPLE_CONTROLLER_ALL_OTHERS

    rclcpp::Time last_odom_time_;           // time of last odometry message
    DronePose last_pose_;                   // pose from last odometry message

    rclcpp::Time curr_target_time_;         // Deadline to hit the current target
    DronePose curr_target_;                 // Current target pose

    // PID controllers
    pid::Controller2 x_controller_{false, 0.1, 0};
    pid::Controller2 y_controller_{false, 0.1, 0};
    pid::Controller2 z_controller_{false, 0.1, 0};
    pid::Controller2 yaw_controller_{true, 0.2, 0};

    void validate_parameters();

  public:
    FlightControllerSimple(rclcpp::Node &node, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr &cmd_vel_pub);

    void _reset() override;

    void _set_target(int target) override;

    bool _odom_callback(const nav_msgs::msg::Odometry::SharedPtr &msg) override;
  };
}
#endif //FLIGHT_CONTROLLER_SIMPLE_HPP
