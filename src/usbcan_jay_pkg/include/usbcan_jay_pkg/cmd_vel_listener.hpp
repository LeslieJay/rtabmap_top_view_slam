/*
 * @Author: wei.canming
 * @Date: 2024-07-19
 * @LastEditors: wei.canming
 * @LastEditTime: 2025-08-04 17:10:02
 * @FilePath: /colcon_ws/src/usbcan_pkg/usbcan/include/usbcan/cmd_vel_listener.hpp
 * @Description: 接收cmd_vel话题，发布DifferentialWheel话题
 * 
 * Copyright (c) 2024, All Rights Reserved. 
 */

#ifndef __CMD_VEL_LISTENER_HPP__
#define __CMD_VEL_LISTENER_HPP__

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "usbcan_jay_pkg/msg/differential_wheel.hpp"
#include <cmath>

// S曲线加速所需的结构体
struct CurveObject {
    double startSpeed; // 初始速度
    double currentSpeed; // 当前速度
    double targetSpeed; // 目标速度
    double stepspeed; // 加速度
    double accRT; // 加速度realtime
    double jerk; // 加加速度,加速度本身变化的快慢
    double speedMax; // 最大速度
    double speedMin; // 最小速度
    uint32_t aTimes; // 调速时间
    uint32_t maxTimes; // 调速跨度
    double flexible; // 曲线拉伸度
};

class CmdVelListener : public rclcpp::Node
{
public:
  CmdVelListener();
  ~CmdVelListener() = default;

private:
  // S曲线相关函数
  void cal_curespta(CurveObject* curve);
  void velocity_curve(CurveObject* curve);
  
  // 回调函数，处理接收到的cmd_vel消息
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  
  // 发布差速轮速度消息
  void publish_wheel_speeds();

  // 订阅者和发布者
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<usbcan_jay_pkg::msg::DifferentialWheel>::SharedPtr wheel_speed_pub_;
  
  // 参数
  double wheel_separation_; // 轮距
  double wheel_radius_;     // 轮半径
  
  // 左右轮速度
  int left_speed_;  // mm/s
  int right_speed_; // mm/s

  CurveObject left_curve_;
  CurveObject right_curve_;

  void init_curve(CurveObject& curve, double jerk, uint32_t maxTimes, double speedMax, double speedMin, double stepspeed, double flexible);
};


#endif // __CMD_VEL_LISTENER_HPP__
