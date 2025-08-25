/*
 * @Author: 自动生成
 * @Date: 2024-07-19
 * @LastEditors: wei.canming
 * @LastEditTime: 2025-07-25 08:58:10
 * @FilePath: /colcon_ws/src/usbcan_pkg/usbcan/src/cmd_vel_listener.cpp
 * @Description: 接收cmd_vel话题，发布DifferentialWheel话题
 * 
 * Copyright (c) 2024, All Rights Reserved. 
 */

#include "cmd_vel_listener.hpp"
#include <cmath>



CmdVelListener::CmdVelListener() : Node("cmd_vel_listener")
{
  // 初始化参数
  this->declare_parameter<double>("wheel_separation", 0.544); // 默认轮距为0.5米
  // this->declare_parameter<double>("wheel_radius", 0.1);     // 默认轮半径为0.1米
  
  this->get_parameter("wheel_separation", wheel_separation_);
  // this->get_parameter("wheel_radius", wheel_radius_);
  
  // 初始化速度
  left_speed_ = 0;
  right_speed_ = 0;

  // 初始化S曲线对象
  // 从param.yaml文件读取S曲线参数
  double left_speed_max, left_speed_min, left_stepspeed, left_flexible, left_jerk;
  double right_speed_max, right_speed_min, right_stepspeed, right_flexible, right_jerk;
  
  this->declare_parameter<double>("left_speed_max", 0.5);
  this->declare_parameter<double>("left_speed_min", -0.5);
  this->declare_parameter<double>("left_stepspeed", 0.1);
  this->declare_parameter<double>("left_flexible", 10.0);
  this->declare_parameter<double>("left_jerk", 1.0);
  this->declare_parameter<int>("left_maxTimes", 100);
  
  this->declare_parameter<double>("right_speed_max", 0.5);
  this->declare_parameter<double>("right_speed_min", -0.5);
  this->declare_parameter<double>("right_stepspeed", 0.1);
  this->declare_parameter<double>("right_flexible", 10.0);
  this->declare_parameter<double>("right_jerk", 1.0);
  this->declare_parameter<int>("right_maxTimes", 100);
  
  this->get_parameter("left_speed_max", left_speed_max);
  this->get_parameter("left_speed_min", left_speed_min);
  this->get_parameter("left_stepspeed", left_stepspeed);
  this->get_parameter("left_flexible",  left_flexible);
  this->get_parameter("left_jerk", left_jerk);
  
  this->get_parameter("right_speed_max", right_speed_max);
  this->get_parameter("right_speed_min", right_speed_min);
  this->get_parameter("right_stepspeed", right_stepspeed);
  this->get_parameter("right_flexible",  right_flexible);
  this->get_parameter("right_jerk", right_jerk);

  uint32_t left_maxTimes = static_cast<uint32_t>(this->get_parameter("left_maxTimes").as_int());
  uint32_t right_maxTimes = static_cast<uint32_t>(this->get_parameter("right_maxTimes").as_int());

  init_curve(left_curve_, left_jerk, left_maxTimes, left_speed_max, left_speed_min, left_stepspeed, left_flexible);
  init_curve(right_curve_, right_jerk, right_maxTimes, right_speed_max, right_speed_min, right_stepspeed, right_flexible);
  
  // 创建订阅者，订阅cmd_vel话题
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, 
    std::bind(&CmdVelListener::cmd_vel_callback, this, std::placeholders::_1));
    
  // 创建发布者，发布差速轮速度
  wheel_speed_pub_ = this->create_publisher<fork_interfaces::msg::DifferentialWheel>(
    "ShaoGuan/AG05D/N38/DifferentialWheel_vel", 10);
    
  RCLCPP_INFO(this->get_logger(), "CmdVelListener已初始化，轮距: %.2f米, 轮半径: %.2f米 \n 左轮最大速度: %.2f m/s, 左轮jerk: %.2f, 左轮最大时间: %d, 最小速度: %.2f m/s, 步长: %.2f m/s, 拉伸度: %.2f", 
              wheel_separation_, wheel_radius_, left_speed_max, left_jerk, left_maxTimes, left_speed_min, left_stepspeed, left_flexible);
  RCLCPP_INFO(this->get_logger(), "CmdVelListener已初始化，轮距: %.2f米, 轮半径: %.2f米 \n 右轮最大速度: %.2f m/s, 右轮jerk: %.2f, 右轮最大时间: %d, 最小速度: %.2f m/s, 步长: %.2f m/s, 拉伸度: %.2f", 
              wheel_separation_, wheel_radius_, right_speed_max, right_jerk, right_maxTimes, right_speed_min, right_stepspeed, right_flexible);
}

void CmdVelListener::init_curve(CurveObject& curve, double jerk, uint32_t maxTimes, double speedMax, double speedMin, double stepspeed, double flexible)
{
  curve.startSpeed = 0.0;
  curve.currentSpeed = 0.0;
  curve.targetSpeed = 0.0;
  curve.stepspeed = stepspeed;
  curve.accRT = 0.0;
  curve.jerk = jerk;
  curve.speedMax = speedMax;
  curve.speedMin = speedMin;
  curve.aTimes = 0;
  curve.maxTimes = maxTimes;
  curve.flexible = flexible;
}
// S曲线计算实时速度
void CmdVelListener::cal_curespta(CurveObject* curve)
{
  // 初始化功率和速度变量
  double power = 0.0;
  double speed = 0.0;
  
  // 计算当前时间在加速周期中的相对位置，范围从-1到1
  power = (2 * ((float)curve->aTimes) - ((float)curve->maxTimes)) / ((float)curve->maxTimes);
  
  // 将power值乘以曲线拉伸系数的负值
  power = (0.0 - curve->flexible) * power;
  
  // 计算S型曲线的值，使用指数函数192.168.3.142
  speed = 1 + exp(power);
  
  // 根据起始速度和目标速度，计算当前应有的速度值
  speed = (curve->targetSpeed - curve->startSpeed) / speed;

  // 计算当前实际速度，即起始速度加上变化量
  curve->currentSpeed = speed + curve->startSpeed;

  // 检查并限制目标速度不超过最大值
  if(curve->targetSpeed > curve->speedMax)
  {
    curve->targetSpeed = curve->speedMax;
  }
  
  // 检查并限制目标速度不低于最小值
  if(curve->targetSpeed < curve->speedMin)
  {
    curve->targetSpeed = curve->speedMin;
  }
}

// 速度控制曲线
void CmdVelListener::velocity_curve(CurveObject* curve)
{
  // 临时变量
  double temp = 0;

  // 保存当前速度和加速度，用于后续计算加加速度
  double previous_v = curve->currentSpeed;
  double previous_acc = curve->accRT;

  // 确保目标速度不超过最大限制
  if(curve->targetSpeed > curve->speedMax)
  {
     curve->targetSpeed = curve->speedMax;
  }
  
  // 确保目标速度不低于最小限制
  if(curve->targetSpeed < curve->speedMin)
  {
     curve->targetSpeed = curve->speedMin;
  }
 
  // 当首次开始加速或需要重新计算加速周期时执行
  if((fabs(curve->currentSpeed - curve->startSpeed) <= curve->stepspeed) && (curve->maxTimes == 0))
  {
     // 确保起始速度不低于最小速度
     if(curve->startSpeed < curve->speedMin)
     {
       curve->startSpeed = curve->speedMin;
     }
     
     // 计算从当前速度到目标速度需要的步数
     temp = fabs(curve->targetSpeed - curve->startSpeed);
     temp = temp / curve->stepspeed;
     temp = std::trunc(temp); // 取整
     curve->maxTimes = (uint32_t)(temp) + 1;
     curve->aTimes = 0;
  }
  
  // 如果还在加速过程中
  if(curve->aTimes < curve->maxTimes)
  {
    // 调用S曲线计算当前速度
    cal_curespta(curve);
    curve->aTimes++;
    // 计算实时加速度和加加速度
    curve->accRT = (curve->currentSpeed - previous_v);
    curve->jerk = (curve->accRT - previous_acc);
  }
  else
  {
     // 达到目标速度，结束加速过程
     curve->currentSpeed = curve->targetSpeed;
     curve->maxTimes = 0;
     curve->aTimes = 0;
  }
}

void CmdVelListener::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // 根据运动学模型计算左右轮速度
  // v = (v_right + v_left) / 2
  // w = (v_right - v_left) / wheel_separation
  
  double linear_velocity = msg->linear.x;   // 线速度 m/s
  double angular_velocity = msg->angular.z; // 角速度 rad/s

  RCLCPP_INFO(this->get_logger(), "接收到cmd_vel: 线速度=%.2f m/s, 角速度=%.2f rad/s", 
               msg->linear.x, msg->angular.z);
  
  // 计算左右轮速度 (m/s)
  double v_left = linear_velocity - (wheel_separation_ * angular_velocity) / 2.0;
  double v_right = linear_velocity + (wheel_separation_ * angular_velocity) / 2.0;
  
  RCLCPP_INFO(this->get_logger(), "期望左右轮速度分别为: v_left=%.2f m/s, v_right=%.2f m/s", v_left, v_right);

  // 每次回调时，更新startSpeed为上一次currentSpeed，保证加速曲线推进
  left_curve_.startSpeed = left_curve_.currentSpeed;
  right_curve_.startSpeed = right_curve_.currentSpeed;

  // 只更新目标速度
  left_curve_.targetSpeed = v_left;
  right_curve_.targetSpeed = v_right;
  
  // 计算平滑轮速
  velocity_curve(&left_curve_);
  velocity_curve(&right_curve_);
  
  // 转换为mm/s并四舍五入为整数
  left_speed_ = static_cast<int>(std::round(left_curve_.currentSpeed * 1000.0));
  right_speed_ = static_cast<int>(std::round(right_curve_.currentSpeed * 1000.0));
  
  RCLCPP_DEBUG(this->get_logger(), "接收到cmd_vel: 线速度=%.2f m/s, 角速度=%.2f rad/s", 
               linear_velocity, angular_velocity);
               
  // 发布轮速消息
  publish_wheel_speeds();
}

void CmdVelListener::publish_wheel_speeds()
{
  auto msg = fork_interfaces::msg::DifferentialWheel();
  msg.left_speed = left_speed_;
  msg.right_speed = right_speed_;
  
  wheel_speed_pub_->publish(msg);
  RCLCPP_DEBUG(this->get_logger(), "发布轮速: 左轮=%d mm/s, 右轮=%d mm/s", 
               left_speed_, right_speed_);
  RCLCPP_INFO(this->get_logger(), "发布轮速: 左轮=%d mm/s, 右轮=%d mm/s", 
               left_speed_, right_speed_);
}

// 入口函数
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
