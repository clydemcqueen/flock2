#ifndef FILTER_NODE_H
#define FILTER_NODE_H

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "kf.hpp"

namespace filter_node {

class FilterNode : public rclcpp::Node
{
  // Parameters
  std::string map_frame_;
  std::string base_frame_;

  // Pose of base_frame in camera_frame
  tf2::Transform t_camera_base_;

  // Node state
  bool mission_;
  rclcpp::Time prev_stamp_;
  kf::KalmanFilter filter_;
  nav_msgs::msg::Path path_msg_;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_mission_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr camera_pose_sub_;

  // Publications
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

  void start_mission_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void stop_mission_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void camera_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

public:

  explicit FilterNode();
  ~FilterNode() {}
};

} // namespace filter_node

#endif // FILTER_NODE_H
