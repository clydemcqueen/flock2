#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/aruco.hpp"

namespace usb_camera {

class USBCamera : public rclcpp::Node
{
public:

  explicit USBCamera() : Node("usb_camera"), stream1(0)
  {
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 1);

    if (!stream1.isOpened()) {
      throw "cannot open camera";
    }
  }

  ~USBCamera() {}

  void spin()
  {
    cv::Mat frame;
    std_msgs::msg::Header header{};
    header.frame_id = "camera_frame";

    while (rclcpp::ok()) {
      // Block until a frame is available
      stream1.read(frame);

      cv::imshow("raw", frame);
      cv::waitKey(1);

      // Publish image_raw if there are subscribers
      if (count_subscribers(image_pub_->get_topic_name()) > 0) {
        header.stamp = now();
        cv_bridge::CvImage image{header, sensor_msgs::image_encodings::BGR8, frame};
        image_pub_->publish(image.toImageMsg());
      }

      // Publish camera_info
      // TODO
    }
  }

private:

  cv::VideoCapture stream1;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
};

} // namespace usb_camera

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<usb_camera::USBCamera>();
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

  // Spin until rclcpp::ok() returns false
  node->spin();

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
