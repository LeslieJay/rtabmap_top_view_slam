/*
 * @Author: LiFang6606397
 * @Date: 2024-02-26 16:52:45
 * @LastEditors: wei.canming
 * @LastEditTime: 2024-12-31 14:21:18
 * @FilePath: /colcon_ws/src/usbcan/include/usbcan/usbcan_parser.hpp
 * @Description: 
 *    input : CAN 
 *    output : 里程计, tf变换, 轮子的转速, 电池信息
 * 
 * Copyright (c) 2024 by LiFang6606397, All Rights Reserved. 
 */

#ifndef __USBCAN_PARSER__H__
#define __USBCAN_PARSER__H__


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <queue>
#include <mutex>
#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ref_slam_interface/action/ctrl_fork.hpp"
#include "usbcan_utils.hpp"
#include "fork_interfaces/msg/differential_wheel.hpp"

#define MAX_CHANNELS  2
#define CHECK_POINT  200
#define RX_WAIT_TIME  10
#define RX_BUFF_SIZE  100

#define SAMPLING_INTERVAL 60
// #define WHEELBASE 1.1

#define PI 3.14159265358979323846

/** pose of base_link */
extern double distance_x;
extern double distance_y;
extern double theta_k;

/** 货叉当前高度 */
extern double fork_height_;

/** 舵轮当前姿态(角度) */
extern double steer_ang_;

extern std::mutex que_mtx_;
extern bool ctl_queue_;

class CANParser : public rclcpp::Node{
public:
  explicit CANParser(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CANParser();

  enum LiftMode{
    RUN_AND_LIFT,
    STOP_AND_LIFT
  };
  
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr 
  getPub() const {
    return publisher_;
  }
  
  std::shared_ptr<tf2_ros::TransformBroadcaster> 
  getTF() const {
    return tf_broadcaster_;
  }

  std::shared_ptr<std::queue<GeneralFrame>>
  getQ() const {
    return cmd_queue_;
  }

private:

  // rclcpp::Time timestamp_;

  // 处理接收到的目标值
  rclcpp_action::GoalResponse 
  handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ref_slam_interface::action::CtrlFork::Goal> goal);
  
  // 处理取消任务
  rclcpp_action::CancelResponse 
  handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<ref_slam_interface::action::CtrlFork>> goal_handle);

  // 产生连续反馈和最终结果
  void 
  execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<ref_slam_interface::action::CtrlFork>> goal_handle);
  
  void 
  handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<ref_slam_interface::action::CtrlFork>> goal_handle);


  void topic_callback(const fork_interfaces::msg::DifferentialWheel::ConstSharedPtr msg);

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<fork_interfaces::msg::DifferentialWheel>::SharedPtr subscript_;
  rclcpp_action::Server<ref_slam_interface::action::CtrlFork>::SharedPtr server_;

  LiftMode liftmode_ = LiftMode::RUN_AND_LIFT;
  bool is_fork_action_running_ = false;
  std::shared_ptr<std::queue<GeneralFrame>> cmd_queue_;
};


#endif  //!__USBCAN_PARSER__H__




