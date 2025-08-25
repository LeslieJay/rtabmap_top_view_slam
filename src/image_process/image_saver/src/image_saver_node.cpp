/*
 * @Autor: wei.canming
 * @Version: 1.0
 * @Date: 2025-01-20 11:26:47
 * @LastEditors: wei.canming
 * @LastEditTime: 2025-04-28 13:28:03
 * @Description: 将深度图resize成彩色图的尺寸对齐后再分别保存
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace fs = std::filesystem;

class ImageSaverNode : public rclcpp::Node
{
public:
    ImageSaverNode() : Node("image_saver_node")
    {   
        // 订阅的彩色图和深度图话题名称 topic
        this->declare_parameter<std::string>("color_topic", "/camera/color/image_raw");
        this->declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");

        // 处理后的深度图与彩色图将要保存的文件夹路径
        this->declare_parameter<std::string>("depth_save_dir", "/home/huanyue/weicanming/GitLabProjects/rtabmap_vslam/top-view-vslam/rtabmap_ws/saved_images/depth");
        this->declare_parameter<std::string>("color_save_dir", "/home/huanyue/weicanming/GitLabProjects/rtabmap_vslam/top-view-vslam/rtabmap_ws/saved_images/color");
        // fs::path currentFilePath(__FILE__);
        // fs::path currentFolderPath = fs::canonical(currentFilePath.parent_path()); 

        // fs::path depth_save_path = currentFolderPath / "saved_images" / "depth";
        // std::string depth_save_dir_ = depth_save_path.string();

        // fs::path color_save_path = currentFolderPath / "saved_images" / "color";
        // std::string color_save_dir_ = color_save_path.string();
        


        // 设置采样频率为3s一张
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(300), 
            [this]() {
                if (lastest_color) {
                    this->color_process();
                }
                if (lastest_depth) {
                    this->depth_process();
                }
            });

        color_topic_ = this->get_parameter("color_topic").as_string();
        depth_topic_ = this->get_parameter("depth_topic").as_string();
        color_save_dir_ = this->get_parameter("color_save_dir").as_string();
        depth_save_dir_ = this->get_parameter("depth_save_dir").as_string();

        // 创建保存目录
        fs::create_directories(color_save_dir_);
        fs::create_directories(depth_save_dir_);

        // 设置QoS策略，限制队列深度为1
        rclcpp::QoS qos(1);

        color_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            color_topic_, qos, std::bind(&ImageSaverNode::color_callback, this, std::placeholders::_1));

        depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic_, 1, std::bind(&ImageSaverNode::depth_callback, this, std::placeholders::_1));

        // timer_ = this->create_wall_timer(std::chrono::milliseconds(3000), std::bind(&ImageSaverNode::color_process, this));

        RCLCPP_INFO(this->get_logger(), "Subscribed to color topic: %s", color_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribed to depth topic: %s", depth_topic_.c_str());
        // RCLCPP_INFO(this->get_logger(), "Images will be saved to: %s", save_dir_.c_str());
    }

private:
    void color_callback(const sensor_msgs::msg::Image::SharedPtr msg){
        lastest_color = msg;
    }
    /**
     * @brief 彩色图话题的回调函数
     * @details 不做处理，直接保存
     */
    void color_process()
    {
        try
        {
            auto cv_image = cv_bridge::toCvCopy(lastest_color, "8UC3")->image;
            std::string filename = generate_filename(color_save_dir_,"color");
            cv::imwrite(filename, cv_image);
            RCLCPP_INFO(this->get_logger(), "Saved color image to %s", filename.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to process color image: %s", e.what());
        }
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg){
        lastest_depth = msg;
    }
    //! TM265相机的深度图保存方式
    void depth_process()
    {
        RCLCPP_INFO(this->get_logger(), "Received depth image with encoding: %s", lastest_depth->encoding.c_str());
        auto cv_image = cv_bridge::toCvCopy(lastest_depth, "16UC1")->image;
        cv::Mat resized_image;
        int target_width_ = 1280;
        int target_height_ = 720;
        cv::resize(cv_image, resized_image, cv::Size(target_width_, target_height_));
        std::string filename = generate_filename(depth_save_dir_, "depth");
        cv::imwrite(filename, resized_image);
        RCLCPP_INFO(this->get_logger(), "Saved depth image to %s", filename.c_str());
    }


    //! TM461相机的深度图保存方式
    /**
     * @brief 深度图话题的回调函数
     * @details 改变深度图的图像格式，再resize深度图，归一化映射后进行保存
     */
    // void depth_process()
    // {
    //     try
    //     {   
    //         RCLCPP_INFO(this->get_logger(), "Received depth image with encoding: %s", lastest_depth->encoding.c_str());
    //         auto cv_image = cv_bridge::toCvCopy(lastest_depth, "16UC1")->image;
    //         std::string filename = generate_filename(depth_save_dir_, "depth");

    //         // 调整图像大小
    //         cv::Mat resized_image;
    //         int target_width_ = 1920;
    //         int target_height_ = 1080;
    //         cv::resize(cv_image, resized_image, cv::Size(target_width_, target_height_));
    //         // 归一化深度图并应用颜色映射
    //         cv::Mat depthNormalized;
    //         cv::normalize(resized_image, depthNormalized, 0, 255, cv::NORM_MINMAX, CV_8U);
    //         cv::Mat depthColormap;
    //         cv::applyColorMap(depthNormalized, depthColormap, cv::COLORMAP_JET); // 热力图颜色


    //         cv::imwrite(filename, cv_image);
    //         RCLCPP_INFO(this->get_logger(), "Saved depth image to %s", filename.c_str());
    //     }
    //     catch (const std::exception &e)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Failed to process depth image: %s", e.what());
    //     }
    // }
    /**
     * @brief 生成文件保存路径
     * @param save_dir 路径字符串
     * @param prefix 前缀，区分depth/color
     */
    std::string generate_filename(const std::string &save_dir, const std::string &prefix)
    {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << save_dir << "/" << prefix << "_"
           << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S")
           << "_" << std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000
           << ".png";
        return ss.str();
    }

    std::string color_topic_;
    std::string depth_topic_;
    std::string color_save_dir_;
    std::string depth_save_dir_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::Image::SharedPtr lastest_color;
    sensor_msgs::msg::Image::SharedPtr lastest_depth;
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSaverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
