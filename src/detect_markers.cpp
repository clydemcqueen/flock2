#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/core.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/highgui.hpp"

namespace detect_markers {

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
  }

  ~DetectMarkers() {}

  // ROS subscriptions
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_sub_;

  // ROS publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_markers_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_marked_pub_;

private:

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    (void)msg;
    // TODO
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert ROS to OpenCV
    auto copy = cv_bridge::toCvCopy(msg);
    cv::Mat another_copy = copy->image;

    // Detect markers
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(another_copy, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    // Draw detected markers
    cv::aruco::drawDetectedMarkers(another_copy, markerCorners, markerIds);

    // Display result
    cv::imshow("marked", another_copy);
    cv::waitKey(1);

    // Publish result
    if (count_subscribers(image_marked_pub_->get_topic_name()) > 0) {
      image_marked_pub_->publish(copy->toImageMsg());
    }

    // Compute pose
    std::vector<cv::Vec3d> rvecs, tvecs;
    // TODO

    // Publish pose
    // TODO
  }
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