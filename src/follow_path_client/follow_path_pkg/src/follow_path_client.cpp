/*
 * @Autor: wei.canming
 * @Version: 1.0
 * @Date: 2025-07-25 16:22:11
 * @LastEditors: wei.canming
 * @LastEditTime: 2025-07-25 16:22:51
 * @Description: 
 */
#include <memory>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class FollowPathClient : public rclcpp::Node
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

  FollowPathClient() : Node("follow_path_client")
  {
    this->declare_parameter<std::string>("frame_id", "map");
    this->get_parameter("frame_id", frame_id_);

    action_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");
    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                     std::bind(&FollowPathClient::send_goal, this));
  }

private:
  void send_goal()
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_WARN(this->get_logger(), "Controller server not available");
      return;
    }

    auto goal_msg = FollowPath::Goal();
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id_;
    path.header.stamp = this->now();

    for (int i = 0; i < 5; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = i * 0.5;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }

    goal_msg.path = path;
    goal_msg.controller_id = "";
    goal_msg.goal_checker_id = "";

    RCLCPP_INFO(this->get_logger(), "Sending path with %zu poses...", path.poses.size());

    auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&FollowPathClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback =
      std::bind(&FollowPathClient::result_callback, this, std::placeholders::_1);

    action_client_->async_send_goal(goal_msg, send_goal_options);

    timer_->cancel();  // 只发送一次
  }

  void goal_response_callback(GoalHandleFollowPath::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal rejected by controller server.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by controller server.");
    }
  }

  void result_callback(const GoalHandleFollowPath::WrappedResult & result)
  {
    RCLCPP_INFO(this->get_logger(), "FollowPath finished with result code: %d", result.code);
  }

  std::string frame_id_;
  rclcpp_action::Client<FollowPath>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowPathClient>());
  rclcpp::shutdown();
  return 0;
}
