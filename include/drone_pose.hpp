#ifndef DRONE_POSE_H
#define DRONE_POSE_H

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace drone_base {

//=============================================================================
// Utilities
//=============================================================================

// Move an angle to the region [-M_PI, M_PI]
double norm_angle(double a)
{
  while (a < -M_PI)
  {
    a += 2 * M_PI;
  }
  while (a > M_PI)
  {
    a -= 2 * M_PI;
  }

  return a;
}

// Compute a 2d point in a rotated frame (v' = R_transpose * v)
void rotate_frame(const double x, const double y, const double theta, double &x_r, double &y_r)
{
  x_r = x * cos(theta) + y * sin(theta);
  y_r = y * cos(theta) - x * sin(theta);
}

//=====================================================================================
// 4 DoF drone pose, in the world frame
//=====================================================================================

struct DronePose
{
  double x;
  double y;
  double z;
  double yaw;

  constexpr DronePose(): x(0), y(0), z(0), yaw(0) {}

  void fromMsg(const geometry_msgs::msg::Pose &msg)
  {
    x = msg.position.x;
    y = msg.position.y;
    z = msg.position.z;

    // Quaternion to yaw
    tf2::Quaternion q;
    tf2::fromMsg(msg.orientation, q);
    double roll = 0, pitch = 0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  }

  void toMsg(geometry_msgs::msg::Pose &msg) const
  {
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;

    // Yaw to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    msg.orientation = tf2::toMsg(q);
  }

  // Within some epsilon
  bool close_enough(const DronePose &that) const
  {
    const double EPSILON_XYZ = 0.25;
    const double EPSILON_YAW = 0.25;

    return
      std::abs(x - that.x) < EPSILON_XYZ &&
      std::abs(y - that.y) < EPSILON_XYZ &&
      std::abs(z - that.z) < EPSILON_XYZ &&
      std::abs(yaw - that.yaw) < EPSILON_YAW;
  }
};

} // namespace drone_base

#endif // DRONE_POSE_H