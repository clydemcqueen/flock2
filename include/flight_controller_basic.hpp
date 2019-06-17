#ifndef FLIGHT_CONTROLLER_BASIC_HPP
#define FLIGHT_CONTROLLER_BASIC_HPP

#include "context_macros.hpp"
#include "flight_controller_interface.hpp"
#include "pid.hpp"

namespace drone_base
{

#define BASIC_CONTROLLER_ALL_PARAMS \
  CXT_MACRO_MEMBER(               /* Allow drone to stabilize for this duration */ \
  stabilize_time_sec, \
  double, 5.) \
  /* End of list */

#define BASIC_CONTROLLER_ALL_OTHERS \
  CXT_MACRO_MEMBER(             /* Allow drone to stabilize for this duration */ \
  stabilize_time,  \
  rclcpp::Duration, 0) \
  /* End of list */

  class FlightControllerBasic : public FlightControllerInterface
  {

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
    BASIC_CONTROLLER_ALL_PARAMS
    BASIC_CONTROLLER_ALL_OTHERS

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
    FlightControllerBasic(rclcpp::Node &node, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr &cmd_vel_pub);

    void _reset() override;

    void _set_target(int target) override;

    bool _odom_callback(const nav_msgs::msg::Odometry::SharedPtr &msg) override;
  };

}

#endif //FLIGHT_CONTROLLER_BASIC_HPP
