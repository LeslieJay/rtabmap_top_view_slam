#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h> // 用于OpenCV和ROS图像消息的转换
#include <opencv2/opencv.hpp>    // OpenCV库
#include "nav_msgs/msg/odometry.hpp"


using namespace std::chrono_literals;


class ImageProcessor : public rclcpp::Node
{
public:
  ImageProcessor()
  : Node("image_processor")
  {
    // 创建订阅者
    color_image_sub_.subscribe(this, "camera/color/image_raw");
    depth_image_sub_.subscribe(this, "camera/depth/image_raw");
    camera_info_sub_.subscribe(this, "camera/color/camera_info");
    odom_msg_sub_.subscribe(this, "/odom");

    // 使用ApproximateTime策略同步三个话题
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), color_image_sub_, depth_image_sub_, camera_info_sub_, odom_msg_sub_);
    sync_->registerCallback(std::bind(&ImageProcessor::imageCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

    // 创建发布者
    color_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/color/image_processed", 10);
    depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/depth/image_processed", 10);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/color/camera_info_processed", 10);
    odom_msg_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/rtabmap/odom", 10); 
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& color_image,
                     const sensor_msgs::msg::Image::ConstSharedPtr& depth_image,
                     const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info,
                     const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg)
  {
    // RCLCPP_INFO(this->get_logger(), "Published synced messages:");
    // 处理彩色图像：将 "8UC3" 转换为 "rgb8"
    cv_bridge::CvImagePtr cv_color;
    try {
      cv_color = cv_bridge::toCvCopy(color_image, "8UC3"); // 假设输入是BGR格式
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // 将图像编码从 "bgr8" 转换为 "rgb8"
    cv_color->encoding = "rgb8";



    // 发布处理后的彩色图像
    color_image_pub_->publish(*cv_color->toImageMsg());

    // 处理深度图像：调整宽高
    cv_bridge::CvImagePtr cv_depth;
    try {
      cv_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1); // 假设深度图是32位浮点型
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // 调整深度图的宽高
    cv::Size new_size(1280, 720); // 设置新的宽高（示例：640x480）
    cv::Mat resized_depth;
    cv::resize(cv_depth->image, resized_depth, new_size, 0, 0, cv::INTER_LINEAR);
    cv_depth->image = resized_depth;
    
    // 将调整后的深度图发布

    depth_image_pub_->publish(*cv_depth->toImageMsg());

    // 发布相机信息（不做修改）
    auto modified_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
    // modified_msg->height = 96;
    // modified_msg->width = 240;
    camera_info_pub_->publish(*modified_msg);

    odom_msg_pub_->publish(*odom_msg);
    
    RCLCPP_INFO(this->get_logger(), "Depth image timestamp: %f", rclcpp::Time(depth_image->header.stamp).seconds());
    RCLCPP_INFO(this->get_logger(), "Color image timestamp: %f", rclcpp::Time(color_image->header.stamp).seconds());
    RCLCPP_INFO(this->get_logger(), "Camera info timestamp: %f", rclcpp::Time(camera_info->header.stamp).seconds());
    RCLCPP_INFO(this->get_logger(), "Odometry timestamp: %f", rclcpp::Time(odom_msg->header.stamp).seconds());
  }

  // 订阅者
  message_filters::Subscriber<sensor_msgs::msg::Image> color_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_msg_sub_;

  // 同步策略
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, nav_msgs::msg::Odometry> SyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // 发布者
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_msg_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageProcessor>());
  rclcpp::shutdown();
  return 0;
}



