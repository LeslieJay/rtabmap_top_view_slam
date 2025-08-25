/*
 * @Author: LiFang6606397
 * @Date: 2024-06-06 11:34:22
 * @LastEditors: wei.canming
 * @LastEditTime: 2025-04-01 13:35:48
 * @FilePath: /colcon_ws/src/usbcan/src/client.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by wei.canming, All Rights Reserved. 
 */
#include <functional>
#include <future>
#include <memory>
#include <string>

#include "ref_slam_interface/action/ctrl_fork.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace fork_client
{
class ForkActionClient : public rclcpp::Node
{
public:

  using Fork = ref_slam_interface::action::CtrlFork;
  using GoalHandleFork = rclcpp_action::ClientGoalHandle<Fork>;

  ForkActionClient() : Node("fork_action_client"){
    this->client_ptr_ = rclcpp_action::create_client<Fork>(this, "fork_server");

    this->timer_ = this->create_wall_timer(
      std::chrono::seconds(10),
      std::bind(&ForkActionClient::send_goal, this));    
  }

  void send_goal()
  {
    using namespace std::placeholders;

    // 等待操作服务器启动
    if (!this->client_ptr_->wait_for_action_server()) {
      // RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    //实例化一个新的 Fork::Goal
    auto goal_msg = Fork::Goal();
    goal_msg.from_ipc_signal2 = 0x03; 
    goal_msg.to_fork_signal = 0x01;
    goal_msg.fork_goal_height = flag_?1100:800;
    flag_ = !flag_;


    // RCLCPP_INFO_STREAM(this->get_logger(), "Sending goal " << goal_msg.fork_goal_height);

    auto send_goal_options = rclcpp_action::Client<Fork>::SendGoalOptions();
    // 设置响应、反馈和结果回调
    send_goal_options.goal_response_callback =
      std::bind(&ForkActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&ForkActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&ForkActionClient::result_callback, this, _1);
    // 将目标goal发送到服务器
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Fork>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool flag_ = false;

  // 当服务器接收并接受目标时，它会向客户端发送响应。
  void goal_response_callback(GoalHandleFork::SharedPtr future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      // RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      // RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  // 假设服务器接受了目标，它将开始处理。对客户端的任何反馈都将由该函数进行
  void feedback_callback(
    GoalHandleFork::SharedPtr,
    const std::shared_ptr<const Fork::Feedback> feedback)
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), "Current height: " << feedback->fork_height);
  }

  // 当服务器完成处理后，它将向客户端返回一个结果。
  void result_callback(const GoalHandleFork::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Result received: " << result.result->finish);
  }
};  // class ForkActionClient

}  // namespace fork_client

int main(int argc, char * argv[]){

  rclcpp::init(argc, argv);

  auto cli = std::make_shared<fork_client::ForkActionClient>();

  rclcpp::spin(cli);

  rclcpp::shutdown();
  return 0;
}