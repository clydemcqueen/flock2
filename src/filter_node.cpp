#include "filter_node.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//==================================================================
// Estimate the current pose and velocity using a Kalman filter
//
// Model pose, velocity and acceleration (18 variables)
// Measure pose (6 variables)
//
// Transformation notation:
//    t_destination_source is a transform
//    vector_destination = t_destination_source * vector_source
//    xxx_f_destination means xxx is expressed in destination frame
//    xxx_pose_f_destination is equivalent to t_destination_xxx
//    t_a_c = t_a_b * t_b_c
//==================================================================

namespace filter_node {

constexpr int STATE_DIM = 18;
constexpr int MEASUREMENT_DIM = 6;

//==================================================================
// Utility functions
//==================================================================

// rclcpp::Time t() initializes nanoseconds to 0
inline bool valid(rclcpp::Time &t) { return t.nanoseconds() > 0; }

// Create measurement matrix
void to_z(const tf2::Transform &in, Eigen::MatrixXd &out)
{
  const tf2::Vector3& z_p = in.getOrigin();
  const tf2::Matrix3x3& z_r = in.getBasis();

  tf2Scalar roll, pitch, yaw;
  z_r.getRPY(roll, pitch, yaw);

  out = Eigen::MatrixXd(MEASUREMENT_DIM, 1);
  out << z_p.x(), z_p.y(), z_p.z(), roll, pitch, yaw;
}

// Create measurement covariance matrix
void to_R(const std::array<double, 36> &in, Eigen::MatrixXd &out)
{
  out = Eigen::MatrixXd(MEASUREMENT_DIM, MEASUREMENT_DIM);
  for (int i = 0; i < MEASUREMENT_DIM; i++) {
    for (int j= 0; j < MEASUREMENT_DIM; j++) {
      out(i, j) = in[i * MEASUREMENT_DIM + j];
    }
  }
}

// Extract pose from state
void x_to_pose(const Eigen::MatrixXd &in, geometry_msgs::msg::Pose &out)
{
  out.position.x = in(0, 0);
  out.position.y = in(1, 0);
  out.position.z = in(2, 0);

  tf2::Matrix3x3 m;
  m.setRPY(in(3, 0), in(4, 0), in(5, 0));

  tf2::Quaternion q;
  m.getRotation(q);

  out.orientation = tf2::toMsg(q);
}

// Extract velocity from state
void x_to_twist(const Eigen::MatrixXd &in, geometry_msgs::msg::Twist &out)
{
  out.linear.x = in(6, 0);
  out.linear.y = in(7, 0);
  out.linear.z = in(8, 0);

  out.angular.x = in(9, 0);
  out.angular.y = in(10, 0);
  out.angular.z = in(11, 0);
}

// Extract pose covariance from state covariance
void P_to_pose(const Eigen::MatrixXd &in, std::array<double, 36> &out)
{
  for (int i = 0; i < MEASUREMENT_DIM; i++) {
    for (int j= 0; j < MEASUREMENT_DIM; j++) {
      out[i * MEASUREMENT_DIM + j] = in(i, j);  // [0:6, 0:6]
    }
  }
}

// Extract velocity covariance from state covariance
void P_to_twist(const Eigen::MatrixXd &in, std::array<double, 36> &out)
{
  for (int i = 0; i < MEASUREMENT_DIM; i++) {
    for (int j= 0; j < MEASUREMENT_DIM; j++) {
      out[i * MEASUREMENT_DIM + j] = in(i + MEASUREMENT_DIM, j + MEASUREMENT_DIM);  // [6:12, 6:12]
    }
  }
}

// Extract pose from state
void x_to_pose(const Eigen::MatrixXd &in, geometry_msgs::msg::Vector3 &out_t, geometry_msgs::msg::Quaternion &out_q)
{
  out_t.x = in(0, 0);
  out_t.y = in(1, 0);
  out_t.z = in(2, 0);

  tf2::Matrix3x3 m;
  m.setRPY(in(3, 0), in(4, 0), in(5, 0));

  tf2::Quaternion q;
  m.getRotation(q);

  out_q = tf2::toMsg(q);
}

//==================================================================
// Class FilterNode
//==================================================================

FilterNode::FilterNode() :
  Node{"filter_node"},
  mission_{false},
  filter_{STATE_DIM, MEASUREMENT_DIM}
{
  // Get parameters
  get_parameter_or<std::string>("map_frame", map_frame_, "map");
  get_parameter_or<std::string>("base_frame", base_frame_, "base_link");

  // Pose of base_frame in camera_frame
  t_camera_base_ = tf2::Transform(tf2::Quaternion(-0.5, 0.5, -0.5, 0.5), tf2::Vector3(0.035, 0., 0.)).inverse();

  // Measurement function
  Eigen::MatrixXd H(MEASUREMENT_DIM, STATE_DIM);
  H <<
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  filter_.set_H(H);

  // Process noise
  filter_.set_Q(Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 0.0001);

  // Initialize path
  path_msg_.header.frame_id = map_frame_;

  // Callbacks
  auto start_mission_cb = std::bind(&FilterNode::start_mission_callback, this, std::placeholders::_1);
  auto stop_mission_cb = std::bind(&FilterNode::stop_mission_callback, this, std::placeholders::_1);
  auto pose_cb = std::bind(&FilterNode::camera_pose_callback, this, std::placeholders::_1);

  // Subscriptions
  start_mission_sub_ = create_subscription<std_msgs::msg::Empty>("/start_mission", start_mission_cb);
  stop_mission_sub_ = create_subscription<std_msgs::msg::Empty>("/stop_mission", stop_mission_cb);
  camera_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("camera_pose", pose_cb);

  // Publications
  filtered_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("filtered_odom", 1);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("estimated_path", 1);
  tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 1);

  RCLCPP_INFO(get_logger(), "filter_node ready");
}

// Start mission
void FilterNode::start_mission_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  path_msg_.poses.clear();
  mission_ = true;
}

// Stop mission
void FilterNode::stop_mission_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  mission_ = false;
}

// Process raw camera pose and publish estimated drone pose
void FilterNode::camera_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  rclcpp::Time stamp(msg->header.stamp);

  // First message sets the starting time
  if (!valid(prev_stamp_)) {
    RCLCPP_INFO(get_logger(), "receiving poses");
    prev_stamp_ = stamp;
    return;
  }

  // Camera pose
  tf2::Transform t_map_camera;
  tf2::fromMsg(msg->pose.pose, t_map_camera);

  // Drone pose
  tf2::Transform t_map_base = t_map_camera * t_camera_base_;

  // Negative dt can happen during some testing situations
  auto dt = (stamp - prev_stamp_).seconds();
  prev_stamp_ = stamp;
  if (dt < 0) {
    RCLCPP_ERROR(get_logger(), "time went backwards, ignoring message");
    return;
  }

  // Transfer function
  auto dt2 = 0.5 * dt * dt;
  Eigen::MatrixXd F(STATE_DIM, STATE_DIM);
  F <<
    1, 0, 0, 0, 0, 0,    dt, 0, 0, 0, 0, 0,    dt2, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,    0, dt, 0, 0, 0, 0,    0, dt2, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,    0, 0, dt, 0, 0, 0,    0, 0, dt2, 0, 0, 0,
    0, 0, 0, 1, 0, 0,    0, 0, 0, dt, 0, 0,    0, 0, 0, dt2, 0, 0,
    0, 0, 0, 0, 1, 0,    0, 0, 0, 0, dt, 0,    0, 0, 0, 0, dt2, 0,
    0, 0, 0, 0, 0, 1,    0, 0, 0, 0, 0, dt,    0, 0, 0, 0, 0, dt2,

    0, 0, 0, 0, 0, 0,    1, 0, 0, 0, 0, 0,     dt, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,    0, 1, 0, 0, 0, 0,     0, dt, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,    0, 0, 1, 0, 0, 0,     0, 0, dt, 0, 0, 0,
    0, 0, 0, 0, 0, 0,    0, 0, 0, 1, 0, 0,     0, 0, 0, dt, 0, 0,
    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 1, 0,     0, 0, 0, 0, dt, 0,
    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 1,     0, 0, 0, 0, 0, dt,

    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 1;
  filter_.set_F(F);

  // Predict step
  filter_.predict();

  // Update step
  Eigen::MatrixXd z, R;
  to_z(t_map_base, z);
  to_R(msg->pose.covariance, R);
  filter_.update(z, R);

  // Publish odometry with both pose and twist
  if (count_subscribers(filtered_odom_pub_->get_topic_name()) > 0) {
    nav_msgs::msg::Odometry filtered_odom_msg;
    filtered_odom_msg.header = msg->header;
    filtered_odom_msg.child_frame_id = base_frame_;
    x_to_pose(filter_.x(), filtered_odom_msg.pose.pose);
    P_to_pose(filter_.P(), filtered_odom_msg.pose.covariance);
    x_to_twist(filter_.x(), filtered_odom_msg.twist.twist);
    P_to_twist(filter_.P(), filtered_odom_msg.twist.covariance);
    filtered_odom_pub_->publish(filtered_odom_msg);
  }

  // Publish path
  if (mission_ && count_subscribers(path_pub_->get_topic_name()) > 0) {
    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header.stamp = msg->header.stamp;
    pose_stamped_msg.header.frame_id = base_frame_;
    x_to_pose(filter_.x(), pose_stamped_msg.pose);
    path_msg_.poses.push_back(pose_stamped_msg);
    path_pub_->publish(path_msg_);
  }

  // Publish tf
  if (count_subscribers(tf_pub_->get_topic_name()) > 0) {
    geometry_msgs::msg::TransformStamped transform_stamped_msg;
    transform_stamped_msg.header = msg->header;
    transform_stamped_msg.child_frame_id = base_frame_;
    x_to_pose(filter_.x(), transform_stamped_msg.transform.translation, transform_stamped_msg.transform.rotation);
    tf2_msgs::msg::TFMessage tf_msg;
    tf_msg.transforms.push_back(transform_stamped_msg);
    tf_pub_->publish(tf_msg);
  }
}

} // namespace filter_node

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<filter_node::FilterNode>();

  // Spin node
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}