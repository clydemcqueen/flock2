#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/aruco.hpp"

#include "get_camera_info.cpp"

namespace usb_camera {

// Target fps = 30 / (SKIP + 1)
const int SKIP = 0;

class USBCamera : public rclcpp::Node
{
public:

  explicit USBCamera() : Node("usb_camera"), camera_(0)
  {
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 1);

    if (!camera_.isOpened()) {
      RCLCPP_ERROR(get_logger(), "Cannot open camera");
    }

    if (!get_camera_info(camera_info_msg_)) {
      RCLCPP_ERROR(get_logger(), "Cannot get camera info");
    }

    header_.frame_id = "camera_frame";
  }

  ~USBCamera() {}

  void spin()
  {
    cv::Mat frame;

    while (rclcpp::ok()) {
      // Block until a frame is available
      camera_.read(frame);

      // Skip some frames while debugging to slow down the pipeline
      static int skip_count = 0;
      if (++skip_count < SKIP) continue;
      skip_count = 0;

      // Debugging
      cv::imshow("raw", frame);
      cv::waitKey(1);

      // Synchronize messages
      auto stamp = now();

      // Publish image_raw
      if (count_subscribers(image_pub_->get_topic_name()) > 0) {
        header_.stamp = stamp;
        cv_bridge::CvImage image{header_, sensor_msgs::image_encodings::BGR8, frame};
        image_pub_->publish(*image.toImageMsg());
      }

      // Publish camera_info
      if (count_subscribers(camera_info_pub_->get_topic_name()) > 0) {
        camera_info_msg_.header.stamp = stamp;
        camera_info_pub_->publish(camera_info_msg_);
      }
    }
  }

private:

  cv::VideoCapture camera_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  std_msgs::msg::Header header_{};

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

  // Spin until rclcpp::ok() returns false
  node->spin();

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}
