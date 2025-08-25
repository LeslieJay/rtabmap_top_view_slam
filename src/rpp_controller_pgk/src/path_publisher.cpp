/*
 * @Autor: wei.canming
 * @Version: 1.0
 * @Date: 2025-07-18 16:40:59
 * @LastEditors: wei.canming
 * @LastEditTime: 2025-08-05 10:41:30
 * @Description: 
 */
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <cmath>

using namespace std::chrono_literals;
using FollowPath = nav2_msgs::action::FollowPath;
using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

class PathPublisher : public rclcpp::Node
{
public:
    PathPublisher() : Node("path_publisher")
    {
        // 创建action客户端
        action_client_ = rclcpp_action::create_client<FollowPath>(this, "/follow_path");
        
        RCLCPP_INFO(this->get_logger(), "Path Publisher已启动，请输入目标坐标(x y):");
        
        // 启动键盘输入线程
        input_thread_ = std::thread(&PathPublisher::keyboard_input_loop, this);
    }
    
    ~PathPublisher()
    {
        running_ = false;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    void keyboard_input_loop()
    {
        double target_x, target_y;
        
        while (running_ && rclcpp::ok()) {
            std::cout << "\n请输入目标坐标 (x y): ";
            if (std::cin >> target_x >> target_y) {
                RCLCPP_INFO(this->get_logger(), "接收到目标坐标: (%.2f, %.2f)", target_x, target_y);
                send_path_goal(target_x, target_y);
            } else {
                std::cout << "输入格式错误，请重新输入两个数字 (x y)\n";
                std::cin.clear();
                std::cin.ignore(10000, '\n');
            }
        }
    }

    void send_path_goal(double target_x, double target_y)
    {
        // 等待action服务器可用
        if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Action server not available, waiting...");
            return;
        }

        // 创建路径消息
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";

        // 起点（假设机器人在原点）
        double start_x = 0.0;  
        double start_y = 0.0;
        
        // 计算路径点数量，根据距离确定，每0.1米一个点
        double dx = target_x - start_x;
        double dy = target_y - start_y;
        double distance = std::sqrt(dx * dx + dy * dy);
        int num_points = std::max(2, static_cast<int>(distance / 0.1)) + 1;
        
        // 计算朝向角度
        double yaw = std::atan2(dy, dx);
        
        // 创建从起点到终点的直线路径
        for (int i = 0; i < num_points; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = "map";
            
            // 线性插值计算路径点
            double t = static_cast<double>(i) / (num_points - 1);
            pose.pose.position.x = start_x + t * dx;
            pose.pose.position.y = start_y + t * dy;
            pose.pose.position.z = 0.0;
            
            // 设置朝向（朝向目标点）
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);  // roll, pitch, yaw
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
            
            path_msg.poses.push_back(pose);
        }

        // 创建action目标
        auto goal_msg = FollowPath::Goal();
        goal_msg.path = path_msg;
        goal_msg.controller_id = "FollowPath";  // 对应controller_params.yaml中的控制器插件名

        RCLCPP_INFO(this->get_logger(), "发送路径跟踪目标到 (%.2f, %.2f)，包含 %zu 个点", 
                   target_x, target_y, path_msg.poses.size());

        // 发送目标
        auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&PathPublisher::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&PathPublisher::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&PathPublisher::result_callback, this, std::placeholders::_1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandleFollowPath::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleFollowPath::SharedPtr,
        const std::shared_ptr<const FollowPath::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Speed: %.2f m/s", feedback->speed);
    }

    void result_callback(const GoalHandleFollowPath::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Path following succeeded!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Path following was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Path following was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
        std::cout << "\n请输入目标坐标 (x y): ";
    }

    rclcpp_action::Client<FollowPath>::SharedPtr action_client_;
    std::thread input_thread_;
    std::atomic<bool> running_{true};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
