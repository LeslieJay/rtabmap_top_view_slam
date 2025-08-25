/*
 * @Author: LiFang6606397
 * @Date: 2024-02-26 16:52:45
 * @LastEditors: wei.canming
 * @LastEditTime: 2025-08-04 13:19:14
 * @FilePath: /colcon_ws/src/usbcan/src/usbcan_parser.cpp
 * @Description: CANParser类的代码实现
 * 
 * Copyright (c) 2024 by LiFang6606397, All Rights Reserved. 
 */

#include "usbcan_parser.hpp"
#include "usbcan_utils.hpp"
#include "usbcan_jay_pkg/msg/differential_wheel.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;
using usbcan_jay_pkg::msg::DifferentialWheel;

CANParser::CANParser(const rclcpp::NodeOptions & options)
: rclcpp::Node("usbcan_parser", options){
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/wheel", 1);
  subscript_ = this->create_subscription<usbcan_jay_pkg::msg::DifferentialWheel>(
    "ShaoGuan/AG05D/N38/DifferentialWheel_vel", 1, std::bind(&CANParser::topic_callback, this, _1));
  cmd_queue_ = std::make_shared<std::queue<GeneralFrame>>();
}

CANParser::~CANParser(){}

void CANParser::topic_callback(const usbcan_jay_pkg::msg::DifferentialWheel::ConstSharedPtr msg)
{	
	

  double vel_left; //左轮速度 m/s
  double vel_right;

  vel_left = msg->left_speed;
  vel_right = msg->right_speed;

  if(vel_left < 0.0009 && vel_left >= 0) 
  {
    vel_left = 0;
  }
  if(vel_right < 0.0009 && vel_right >= 0) 
  {
    vel_right = 0;
  }


  if(vel_left > 620)
  {
    vel_left = 10;
  }
  if(vel_right > 620)
  {
    vel_right = 10;
  }

  if(vel_left < -620)
  {
    vel_left = -10;
  }
  if(vel_right < -620)
  {
    vel_right = -10;
  }

  // 转化为epec需要的指令格式。单位：mm/s
  // int16_t speed_left_command =  static_cast<int16_t>(std::round(vel_left));
  // int16_t speed_right_command = static_cast<int16_t>(std::round(vel_right)) ;

  int16_t speed_left_command =  static_cast<int16_t>(std::round(vel_left));
  int16_t speed_right_command = static_cast<int16_t>(std::round(vel_right)) ;
  // cout<<"左右轮命令：  "<<speed_left_command<<"     "<<speed_right_command<<endl;
  //  0x264	报文内容为：   A0  86  01  00  46  86  01  00
  //  0A  86  01  00  46  86  01  00 //20241129 调走直后查看的报文内容
  // uint32_t speedScale = 0xA0860100,distanceScale = 0x46860100;
  uint32_t speedScale = 0x0A860100,distanceScale = 0x0A860100;
  // 小端解析：速度和距离系数：  100000 = 1    99910 = 0.9910
  // 速度系数（个人理解）：调走直的关键，当我们使小车走直，命令给与的左右轮速度是一致的，但由于驱动轮本身误差，导致实际上表现的
  // 左右轮速度是不一样的（可能出现左轮大于右轮或右轮大于左轮的情况，导致小车走不直，水平偏差累计增大)。速度系数刚好进行补偿，
  // 使左右轮速度尽可能无限接近一致，使agv整体车身保持直行 。在单舵轮中同样具有转弯系数，单舵轮有一定的角度偏差影响直线行驶  20241129
  // cout<<"速度和距离系数：  "<<std::dec<<speedScale<<"     "<<distanceScale<<endl;
  // 调走直之后 速度系数：0.99850000000000005 
  // 调走直之后距离系数： 0.99909999999999999 
  // AF06只需要发送304  ；AG05D发送304 404 264。关于这两个系数是否需要再此处发送，根据epec烧录的程序而定。这个值可以在调走直之后
  // 获取报文或者直接系数值写死在epec中，这样应该不需要再次发送这两个系数
  cmd_queue_->push(gen_0x264(speedScale , distanceScale ));
  cmd_queue_->push(gen_0x304(speed_left_command,speed_right_command,0,0 ));
  cmd_queue_->push(gen_0x404(0x0007));
  // if(battery_info_.charge_connection == 1){
  //   cmd_queue_->push(gen_0x404(0x0007));

  // }else{
  //   cmd_queue_->push(gen_0x404(0x0003));
  // }

}

