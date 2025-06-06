#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class MonoCameraSimNode : public rclcpp::Node
{
public:
  MonoCameraSimNode()
  : Node("mono_camera_sim_node")
  {
    // create publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/sci_viz_image_raw", 10);

    // Load image from disk
    std::string package_path = ament_index_cpp::get_package_share_directory("avt_vimba_camera");
    std::string image_path = package_path + "/sample_images/test_img.jpg";
    image_ = cv::imread(image_path, cv::IMREAD_COLOR);

    if (image_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load image: %s", image_path.c_str());
      RCLCPP_ERROR(this->get_logger(), "Please check the image path: %s", image_path.c_str());
      return;
    }

    // Timer to publish at 10 Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&MonoCameraSimNode::timer_callback, this)
    );
  }

private:
  void timer_callback()
  {
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_).toImageMsg();
    msg->header.stamp = this->get_clock()->now();
    publisher_->publish(*msg);
    // RCLCPP_INFO(this->get_logger(), "MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM");
    // RCLCPP_INFO(this->get_logger(), "Publishing simulated image at 10 Hz");
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::Mat image_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonoCameraSimNode>());
  rclcpp::shutdown();
  return 0;
}
