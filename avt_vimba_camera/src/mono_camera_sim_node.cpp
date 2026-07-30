#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

// Publishes pre-captured sample images to /sci_viz_image (with matching
// intrinsics on /sci_viz_cam_info) at a configurable rate. Loops through
// every image in sample_images/ (or the directory pointed to by the
// `image_dir` parameter) — intended to stand in for the real AVT mono
// camera in sim mode. Publishes on the rectified topic directly so the
// image_proc rectify node is bypassed (sample images are already usable
// as-is by downstream consumers).
//
// Parameters:
//   image_dir  (string): directory of sample images. Empty (default) uses
//                        share/avt_vimba_camera/sample_images.
//   rate_hz    (double): publish rate. Default 0.5 (one image every 2 s).
//   image_hold_sec (double): how long to keep publishing the same image
//                        before advancing to the next one. Default 10.0.
//   frame_id   (string): frame_id stamped on published messages. Default
//                        "camera_optical_frame".
//   loop       (bool)  : wrap around after the last image. Default true.

class MonoCameraSimNode : public rclcpp::Node
{
public:
  MonoCameraSimNode() : Node("mono_camera_sim_node")
  {
    // Parameters.
    const std::string default_dir =
        ament_index_cpp::get_package_share_directory("avt_vimba_camera") + "/sample_images";
    const std::string image_dir =
        this->declare_parameter<std::string>("image_dir", default_dir);
    const double rate_hz = this->declare_parameter<double>("rate_hz", 0.5);
    image_hold_sec_ = this->declare_parameter<double>("image_hold_sec", 10.0);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "camera_optical_frame");
    loop_ = this->declare_parameter<bool>("loop", true);

    if (rate_hz <= 0.0)
    {
      RCLCPP_ERROR(this->get_logger(), "rate_hz must be > 0, got %f", rate_hz);
      return;
    }
    if (image_hold_sec_ <= 0.0)
    {
      RCLCPP_ERROR(this->get_logger(), "image_hold_sec must be > 0, got %f", image_hold_sec_);
      return;
    }

    // Load all supported images from image_dir (sorted).
    if (!load_images(image_dir))
    {
      RCLCPP_ERROR(this->get_logger(),
                   "No sample images loaded from %s (supported: .jpg/.jpeg/.png/.bmp)",
                   image_dir.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "mono_camera_sim: publishing %zu image(s) from %s at %.3f Hz (hold %.3f s, loop=%s)",
                images_.size(), image_dir.c_str(), rate_hz, image_hold_sec_, loop_ ? "true" : "false");

    // Publishers (sensor-data-ish QoS, matching the original sim node).
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/sci_viz_image", qos);
    cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/sci_viz_cam_info", qos);

    // Timer at the requested rate.
    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&MonoCameraSimNode::publish_next, this));
  }

private:
  static bool is_supported_image(const std::filesystem::path & p)
  {
    static const std::vector<std::string> exts = {".jpg", ".jpeg", ".png", ".bmp"};
    if (!std::filesystem::is_regular_file(p))
    {
      return false;
    }
    std::string ext = p.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return std::find(exts.begin(), exts.end(), ext) != exts.end();
  }

  bool load_images(const std::string & dir)
  {
    std::error_code ec;
    if (!std::filesystem::is_directory(dir, ec))
    {
      return false;
    }
    std::vector<std::filesystem::path> paths;
    for (const auto & entry : std::filesystem::directory_iterator(dir, ec))
    {
      if (is_supported_image(entry.path()))
      {
        paths.push_back(entry.path());
      }
    }
    std::sort(paths.begin(), paths.end());

    for (const auto & p : paths)
    {
      cv::Mat img = cv::imread(p.string(), cv::IMREAD_COLOR);
      if (img.empty())
      {
        RCLCPP_WARN(this->get_logger(), "Failed to read %s, skipping.", p.string().c_str());
        continue;
      }
      // Pre-convert to Image message once so publish_next is cheap.
      std_msgs::msg::Header header;
      header.frame_id = frame_id_;
      auto image_msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();

      // Matching CameraInfo (passthrough intrinsics — sample images are
      // already processed, so this keeps image_proc::RectifyNode a no-op).
      sensor_msgs::msg::CameraInfo info;
      info.header.frame_id = frame_id_;
      info.width = static_cast<uint32_t>(img.cols);
      info.height = static_cast<uint32_t>(img.rows);
      const double fx = 1000.0;
      const double fy = 1000.0;
      const double cx = img.cols / 2.0;
      const double cy = img.rows / 2.0;
      info.k = {fx, 0.0, cx,
                0.0, fy, cy,
                0.0, 0.0, 1.0};
      info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
      info.distortion_model = "plumb_bob";
      info.r = {1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0};
      info.p = {fx, 0.0, cx, 0.0,
                0.0, fy, cy, 0.0,
                0.0, 0.0, 1.0, 0.0};

      images_.push_back({p.filename().string(), image_msg, info});
    }
    return !images_.empty();
  }

  void publish_next()
  {
    if (images_.empty())
    {
      return;
    }

    const auto now = this->get_clock()->now();
    if (!active_sample_initialized_)
    {
      last_switch_time_ = now;
      active_sample_initialized_ = true;
    }

    if ((now - last_switch_time_).seconds() >= image_hold_sec_)
    {
      ++index_;
      if (index_ >= images_.size())
      {
        if (!loop_)
        {
          RCLCPP_INFO(this->get_logger(), "Reached end of sample images (loop=false), cancelling timer.");
          timer_->cancel();
          return;
        }
        index_ = 0;
      }
      last_switch_time_ = now;
    }

    auto & sample = images_[index_];
    const auto stamp = now;

    sample.image_msg->header.stamp = stamp;
    image_pub_->publish(*sample.image_msg);

    sample.info.header.stamp = stamp;
    cam_info_pub_->publish(sample.info);

    RCLCPP_DEBUG(this->get_logger(), "Published %s (%ux%u)",
           sample.name.c_str(), sample.info.width, sample.info.height);
  }

  struct Sample
  {
    std::string name;
    sensor_msgs::msg::Image::SharedPtr image_msg;
    sensor_msgs::msg::CameraInfo info;
  };

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<Sample> images_;
  size_t index_{0};
  rclcpp::Time last_switch_time_{0, 0, RCL_ROS_TIME};
  bool active_sample_initialized_{false};
  double image_hold_sec_{10.0};
  std::string frame_id_;
  bool loop_{true};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonoCameraSimNode>());
  rclcpp::shutdown();
  return 0;
}
