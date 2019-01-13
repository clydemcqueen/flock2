#ifndef FLOCK_BASE_H
#define FLOCK_BASE_H

#include "rclcpp/rclcpp.hpp"

#include "drone.hpp"

namespace flock_base {

// XBox One joystick axes and buttons
const int JOY_AXIS_LEFT_LR = 0;           // Left stick left/right; 1.0 is left and -1.0 is right
const int JOY_AXIS_LEFT_FB = 1;           // Left stick forward/back; 1.0 is forward and -1.0 is back
const int JOY_AXIS_LEFT_TRIGGER = 2;      // Left trigger
const int JOY_AXIS_RIGHT_LR = 3;          // Right stick left/right; 1.0 is left and -1.0 is right
const int JOY_AXIS_RIGHT_FB = 4;          // Right stick forward/back; 1.0 is forward and -1.0 is back
const int JOY_AXIS_RIGHT_TRIGGER = 5;     // Right trigger
const int JOY_AXIS_TRIM_LR = 6;           // Trim left/right; 1.0 for left and -1.0 for right
const int JOY_AXIS_TRIM_FB = 7;           // Trim forward/back; 1.0 for forward and -1.0 for back
const int JOY_BUTTON_A = 0;               // A button
const int JOY_BUTTON_B = 1;               // B button
const int JOY_BUTTON_X = 2;               // X button
const int JOY_BUTTON_Y = 3;               // Y button
const int JOY_BUTTON_LEFT_BUMPER = 4;     // Left bumper
const int JOY_BUTTON_RIGHT_BUMPER = 5;    // Right bumper
const int JOY_BUTTON_VIEW = 6;            // View button
const int JOY_BUTTON_MENU = 7;            // Menu button
const int JOY_BUTTON_LOGO = 8;            // XBox logo button
const int JOY_BUTTON_LEFT_STICK = 9;      // Left stick button
const int JOY_BUTTON_RIGHT_STICK = 10;    // Right stick button

class FlockBase : public rclcpp::Node
{
  // Our drones
  std::vector<std::shared_ptr<Drone>> drones_;

  // Users can use the joystick to manually control one drone at a time
  int manual_control_{0};

  // Joystick assignments
  int joy_axis_throttle_ = JOY_AXIS_RIGHT_FB;
  int joy_axis_strafe_ = JOY_AXIS_RIGHT_LR;
  int joy_axis_vertical_ = JOY_AXIS_LEFT_FB;
  int joy_axis_yaw_ = JOY_AXIS_LEFT_LR;
  int joy_button_takeoff_ = JOY_BUTTON_MENU;
  int joy_button_land_ = JOY_BUTTON_VIEW;
  int joy_button_shift_ = JOY_BUTTON_LEFT_BUMPER;
  int joy_button_stop_mission_ = JOY_BUTTON_A;
  int joy_button_start_mission_ = JOY_BUTTON_B;
  int joy_button_next_drone_ = JOY_BUTTON_RIGHT_BUMPER;
  int joy_axis_trim_lr_ = JOY_AXIS_TRIM_LR;
  int joy_axis_trim_fb_ = JOY_AXIS_TRIM_FB;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

public:

  explicit FlockBase();
  ~FlockBase() {}

  void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg);

  void spin_once();
};
} // namespace flock_base

#endif // FLOCK_BASE_H
