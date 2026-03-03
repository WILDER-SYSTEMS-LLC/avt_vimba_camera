#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class MonoCameraSimNode : public rclcpp::Node
{
public:
  MonoCameraSimNode() : Node("mono_camera_sim_node")
  {
    // create publishers
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/sci_viz_image_raw", 10);
    cam_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/sci_viz_cam_info", 10);

    // Load image from disk
    std::string package_path = ament_index_cpp::get_package_share_directory("avt_vimba_camera");
    std::string image_path = package_path + "/sample_images/test_image.jpg";
    image_ = cv::imread(image_path, cv::IMREAD_COLOR);

    if (image_.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load image: %s", image_path.c_str());
      RCLCPP_ERROR(this->get_logger(), "Check the image path: %s", image_path.c_str());
      return;
    }

    // Setup camera info
    setupCameraInfo();

    // Timer to publish at 10 Hz
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MonoCameraSimNode::timer_callback, this));
  }

private:
  void setupCameraInfo()
  {
    cam_info_.header.frame_id = "camera_optical_frame";
    cam_info_.width = image_.cols;
    cam_info_.height = image_.rows;
    
    // Example camera matrix (intrinsics) - adjust these values as needed
    cam_info_.k = {1000.0, 0.0, static_cast<double>(image_.cols / 2),
                   0.0, 1000.0, static_cast<double>(image_.rows / 2),
                   0.0, 0.0, 1.0};
    
    // Distortion coefficients (k1, k2, t1, t2, k3)
    cam_info_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    cam_info_.distortion_model = "plumb_bob";
    
    // Rectification matrix (identity for monocular camera)
    cam_info_.r = {1.0, 0.0, 0.0,
                   0.0, 1.0, 0.0,
                   0.0, 0.0, 1.0};
    
    // Projection matrix
    cam_info_.p = {1000.0, 0.0, static_cast<double>(image_.cols / 2), 0.0,
                   0.0, 1000.0, static_cast<double>(image_.rows / 2), 0.0,
                   0.0, 0.0, 1.0, 0.0};
  }

  void timer_callback()
  {
    auto stamp = this->get_clock()->now();
    
    // Publish image
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_).toImageMsg();
    msg->header.stamp = stamp;
    publisher_->publish(*msg);
    
    // Publish camera info
    cam_info_.header.stamp = stamp;
    cam_info_publisher_->publish(cam_info_);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::Mat image_;
  sensor_msgs::msg::CameraInfo cam_info_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonoCameraSimNode>());
  rclcpp::shutdown();
  return 0;
}
