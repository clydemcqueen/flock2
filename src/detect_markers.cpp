#include "eigen_util.hpp"

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/aruco.hpp"
#include "opencv2/highgui.hpp"

namespace detect_markers {

// Transformation notation:
// Tst === T_source_target
// vector_target = T_source_target * vector_source

class DetectMarkers : public rclcpp::Node
{
public:

  explicit DetectMarkers() : Node("detect_markers")
  {
    // ROS subscriptions
    using std::placeholders::_1;
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>("camera_info",
      std::bind(&DetectMarkers::camera_info_callback, this, _1));
    image_raw_sub_ = create_subscription<sensor_msgs::msg::Image>("image_raw",
      std::bind(&DetectMarkers::image_callback, this, _1));

    // ROS publishers
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 1);
    rviz_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("rviz_markers", 1);
    tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 1);
    image_marked_pub_ = create_publisher<sensor_msgs::msg::Image>("image_marked", 1);

    // Assume that the 1st marker we see is 1m above the ground
    first_marker_pose_.position.x = 0;
    first_marker_pose_.position.y = 0;
    first_marker_pose_.position.z = 1;
    first_marker_pose_.orientation.x = 0.5;
    first_marker_pose_.orientation.y = -0.5;
    first_marker_pose_.orientation.z = -0.5;
    first_marker_pose_.orientation.w = 0.5;

    // Transform odom to marker
    Tom_ = eigen_util::to_affine(first_marker_pose_).inverse(Eigen::TransformTraits::Isometry);

    // Transform camera to base
    Eigen::Vector3d t(0.035, 0., 0.);
    Eigen::Quaterniond q(0.5, -0.5, 0.5, -0.5); // w, x, y, z
    Tcb_ = Eigen::Affine3d();
    Tcb_.translation() = t;
    Tcb_.linear() = q.toRotationMatrix();

    // Covariance
    // TODO
    covariance_[0] = 3e-4;
    covariance_[7] = 6e-3;
    covariance_[14] = 6e-3;
    covariance_[21] = 1e-4;
    covariance_[28] = 2e-3;
    covariance_[35] = 2e-3;
  }

  ~DetectMarkers() {}

private:

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (!have_camera_info_) {
      camera_matrix_ = cv::Mat(3, 3, CV_64F, 0.);
      camera_matrix_.at<double>(0, 0) = msg->k[0];
      camera_matrix_.at<double>(0, 2) = msg->k[2];
      camera_matrix_.at<double>(1, 1) = msg->k[4];
      camera_matrix_.at<double>(1, 2) = msg->k[5];
      camera_matrix_.at<double>(2, 2) = 1.;

      // ROS and OpenCV (and everybody?) agree on this ordering: k1, k2, t1 (p1), t2 (p2), k3
      dist_coeffs_ = cv::Mat(1, 5, CV_64F);
      dist_coeffs_.at<double>(0) = msg->d[0];
      dist_coeffs_.at<double>(1) = msg->d[1];
      dist_coeffs_.at<double>(2) = msg->d[2];
      dist_coeffs_.at<double>(3) = msg->d[3];
      dist_coeffs_.at<double>(4) = msg->d[4];

      RCLCPP_INFO(get_logger(), "have camera info");
      have_camera_info_ = true;
    }
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg)
  {
    // Convert ROS to OpenCV
    cv_bridge::CvImagePtr color = cv_bridge::toCvCopy(image_msg);

    // Color to gray for detection
    cv::Mat gray;
    cv::cvtColor(color->image, gray, cv::COLOR_BGR2GRAY);

    // Detect markers
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(gray, dictionary_, corners, ids);

    // Stop if no markers were detected
    if (ids.size() == 0) {
      return;
    }

    // Draw detected markers
    cv::aruco::drawDetectedMarkers(color->image, corners, ids);

    if (first_marker_id_ < 0) {
      // Grab the first marker we see
      first_marker_id_ = ids[0];
      RCLCPP_INFO(get_logger(), "first marker has id %d", first_marker_id_);
    } else {
      // Stop if we didn't see the first marker
      bool found_first_marker = false;
      for (int i = 0; i < ids.size(); i++) {
        if (ids[i] == first_marker_id_) {
          found_first_marker = true;
          break;
        }
      }
      if (!found_first_marker) {
        return;
      }
    }

    // Compute marker poses
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

    // Draw poses
    for (int i = 0; i < ids.size(); i++) {
      cv::aruco::drawAxis(color->image, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], 0.1);
    }

    // Publish result
    if (count_subscribers(image_marked_pub_->get_topic_name()) > 0) {
      image_marked_pub_->publish(color->toImageMsg());
    } else {
      cv::imshow("marked", color->image);
      cv::waitKey(1);
    }

    // Compute drone pose
    geometry_msgs::msg::Pose drone_pose;
    Eigen::Affine3d Tbo;
    for (int i = 0; i < ids.size(); i++) {
      if (ids[i] == first_marker_id_) {
        // Tob = Tcb * Tmc * Tom
        Eigen::Affine3d Tmc = eigen_util::to_affine(rvecs[i], tvecs[i]);
        Eigen::Affine3d Tob = Tcb_ * Tmc * Tom_;

        // sendTransform expects child=target and parent=source
        // We can't flip source and target because we want odom to be the parent of base_link
        // Instead, invert Tob to get Tbo
        Tbo = Tob.inverse(Eigen::TransformTraits::Isometry);
        drone_pose = eigen_util::to_pose(Tbo);
      }
    }

    // Compute marker poses
    std::map<int, geometry_msgs::msg::Pose> marker_poses;
    for (int i = 0; i < ids.size(); i++) {
      if (ids[i] == first_marker_id_) {
        // This one is easy
        marker_poses[ids[i]] = first_marker_pose_;
      } else {
        // Compute the pose of this marker
        // Tmo = Tbo * Tcb * Tmc
        Eigen::Affine3d Tmc = eigen_util::to_affine(rvecs[i], tvecs[i]);
        Eigen::Affine3d Tmo = Tbo * Tcb_ * Tmc;
        marker_poses[ids[i]] = eigen_util::to_pose(Tmo);
      }
    }

    // Publish odometry
    if (count_subscribers(odom_pub_->get_topic_name()) > 0) {
      nav_msgs::msg::Odometry odom_msg;
      odom_msg.header.frame_id = "odom";
      odom_msg.header.stamp = image_msg->header.stamp;
      odom_msg.child_frame_id = "base_link";
      odom_msg.pose.pose = drone_pose;

      // Covariance scales w/ distance^3
      auto factor = 1 + std::pow(-drone_pose.position.x - 1, 3);
      for (auto cit = covariance_.begin(), mit = odom_msg.pose.covariance.begin();
          cit != covariance_.end() && mit != odom_msg.pose.covariance.end();
          cit++, mit++) *mit = *cit * factor;

      // Leave odom_msg.twist at 0
      odom_pub_->publish(odom_msg);
    }

    // Publish rviz markers
    if (count_subscribers(rviz_markers_pub_->get_topic_name()) > 0) {
      visualization_msgs::msg::MarkerArray marker_array_msg;
      for (auto tuple: marker_poses) {
        visualization_msgs::msg::Marker marker;
        marker.id = tuple.first;
        marker.header.frame_id = "odom";
        marker.pose = tuple.second;
        marker.type = marker.CUBE;
        marker.action = marker.ADD;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.01;
        marker.color.r = 1.f;
        marker.color.g = 1.f;
        marker.color.b = 0.f;
        marker.color.a = 1.f;
        marker_array_msg.markers.push_back(marker);
      }
      rviz_markers_pub_->publish(marker_array_msg);
    }

    // Publish TF message
    if (count_subscribers(tf_pub_->get_topic_name()) > 0) {
      tf2_msgs::msg::TFMessage tf_msg;
      for (auto tuple: marker_poses) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.header.stamp = image_msg->header.stamp;
        transform_stamped.child_frame_id = std::string("marker") + std::to_string(tuple.first);
        transform_stamped.transform = eigen_util::to_tf(tuple.second);
        tf_msg.transforms.push_back(transform_stamped);
      }
      tf_pub_->publish(tf_msg);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_markers_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_marked_pub_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  bool have_camera_info_{false};
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  float marker_length_{0.18};                   // Markers are 18cm x 18cm
  int first_marker_id_{-1};                     // First marker we find
  geometry_msgs::msg::Pose first_marker_pose_;  // Pose of first marker is fixed
  Eigen::Affine3d Tom_;                         // Transform odom => first marker
  Eigen::Affine3d Tcb_;                         // Transform camera_frame => base_link
  std::array<double, 36> covariance_{0};        // Covariance at 1m
};

} // namespace detect_markers

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<detect_markers::DetectMarkers>();
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

  // Spin until rclcpp::ok() returns false
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}