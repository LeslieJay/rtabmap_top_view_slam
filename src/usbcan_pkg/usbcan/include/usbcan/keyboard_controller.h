/*
 * @Author: du.xiaoying1
 * @Date: 2024-11-07 09:13:29
 * @LastEditors: wei.canming
 * @LastEditTime: 2025-07-18 14:48:54
 * @FilePath: /colcon_ws/src/usbcan/usbcan/include/usbcan/keyboard_controller.h
 * @Description: 
 * 
 * Copyright (c) 2024 by du.xiaoying1 , All Rights Reserved. 
 */
#ifndef DIFFERENTIAL_WHEEL_TELEOP_KEYBOARD_CONTROLLER_H
#define DIFFERENTIAL_WHEEL_TELEOP_KEYBOARD_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include "fork_interfaces/msg/differential_wheel.hpp"  // 替换为实际的消息头文件
#include <fcntl.h>
#include <unistd.h>

struct CurveObject{
double startSpeed; //初始速度
double currentSpeed ;//当前速度
double targetSpeed ;//目标速度
double stepspeed ;//加速度 //!
double accRT;//加速度realtime
double jerk;//加加速度,加速度本身变化的快慢
double speedMax;//最大速度
double speedMin;//最小速度
uint32_t aTimes;    //调速时间
uint32_t maxTimes;   //调速跨度
double flexible;//曲线拉伸度
};

CurveObject cure;

class KeyboardController : public rclcpp::Node
{
public:
  KeyboardController();
  void spin();

private:
  void handle_key(char key);
  void publish_speeds();
  void velocity_curve(CurveObject* curve);
  void cal_curespta(CurveObject* curve);
  void forward_accelerate();
  void back_accelerate();
  void deaccelerate();

  rclcpp::Publisher<fork_interfaces::msg::DifferentialWheel>::SharedPtr speed_pub_;
  int left_speed_, right_speed_;
};

#endif  // DIFFERENTIAL_WHEEL_TELEOP_KEYBOARD_CONTROLLER_H