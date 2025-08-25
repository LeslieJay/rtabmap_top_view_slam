/*
 * @Author: LiFang6606397
 * @Date: 2024-02-26 16:52:45
 * @LastEditors: wei.canming
 * @LastEditTime: 2025-07-24 15:26:28
 * @FilePath: /colcon_ws/src/usbcan/src/usbcan_parser.cpp
 * @Description: CANParser类的代码实现
 * 
 * Copyright (c) 2024 by LiFang6606397, All Rights Reserved. 
 */

#include "usbcan_parser.hpp"
#include "usbcan_utils.hpp"
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;
using fork_interfaces::msg::DifferentialWheel;

CANParser::CANParser(const rclcpp::NodeOptions & options)
: rclcpp::Node("usbcan_parser", options){
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/wheel", 1);
  subscript_ = this->create_subscription<fork_interfaces::msg::DifferentialWheel>(
    "ShaoGuan/AG05D/N38/DifferentialWheel_vel", 1, std::bind(&CANParser::topic_callback, this, _1));
  server_ = rclcpp_action::create_server<ref_slam_interface::action::CtrlFork>(
    this, "fork_server",
    std::bind(&CANParser::handle_goal,this,_1,_2),
    std::bind(&CANParser::handle_cancel,this,_1),
    std::bind(&CANParser::handle_accepted,this,_1));
  cmd_queue_ = std::make_shared<std::queue<GeneralFrame>>();
}

CANParser::~CANParser(){}

void CANParser::topic_callback(const fork_interfaces::msg::DifferentialWheel::ConstSharedPtr msg)
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
// 处理接收到的目标值
rclcpp_action::GoalResponse 
CANParser::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ref_slam_interface::action::CtrlFork::Goal>goal){
    (void)uuid;
    if (goal->to_fork_signal != 1){
        RCLCPP_INFO(this->get_logger(),"To use fork, the to_fork_signal must be 1.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(),"Goal accepted.");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 处理取消任务
rclcpp_action::CancelResponse 
CANParser::handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<ref_slam_interface::action::CtrlFork>> goal_handle){
    (void) goal_handle;
    RCLCPP_INFO(this->get_logger(),"Goal canceled.");
    return rclcpp_action::CancelResponse::ACCEPT;
}

// 产生连续反馈和最终结果
void 
CANParser::execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<ref_slam_interface::action::CtrlFork>> goal_handle){

  is_fork_action_running_ = true;
  // 发送控制货叉指令
  {
    std::lock_guard<std::mutex> lock(que_mtx_);
    cmd_queue_->push(gen_0x204(0x17,0,0x01,0x01));
    // if(liftmode_==LiftMode::STOP_AND_LIFT){
    //   cmd_queue_->push(gen_0x304(0, steer_ang_*100, 65465));  // 保持上一刻的姿态，保持稳定
    // }
    //cmd_queue_->push(gen_0x404(0x03, 0x01, goal_handle->get_goal()->fork_goal_height));
  }
  auto feedback = std::make_shared<ref_slam_interface::action::CtrlFork::Feedback>();
  auto result = std::make_shared<ref_slam_interface::action::CtrlFork::Result>();

  /** TODO: 这里可能导致 距离差在50mm内时，is_fork_action_running_ 标志位变为false，导航舵轮提前发生运动 */
  while(std::abs(goal_handle->get_goal()->fork_goal_height - fork_height_) >= 50 && rclcpp::ok()){
    feedback->fork_height = fork_height_;
    goal_handle->publish_feedback(feedback);

    if(goal_handle->is_canceling()){
      // 发送关闭货叉使能 ----> 队列中放入停止命令
      {
        std::lock_guard<std::mutex> lock(que_mtx_);
        // cmd_queue_->push(gen_0x404(0x03, 0x00, fork_height_));
        RCLCPP_INFO_STREAM(this->get_logger(), "Aborting fork control");
      }
      
      result->finish = false;
      goal_handle->canceled(result);

      is_fork_action_running_ = false;

      RCLCPP_INFO_STREAM(this->get_logger(),"Goal Aborted.");
      return;
    }
    std::this_thread::sleep_for(50ms);
  }

  if(rclcpp::ok()){
    result->finish = true;
    goal_handle->succeed(result);
    RCLCPP_INFO_STREAM(this->get_logger(),"Goal succeed. Current heght: " << fork_height_);
  }
  is_fork_action_running_ = false;
  return;
}

void 
CANParser::handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<ref_slam_interface::action::CtrlFork>> goal_handle){ 
    std::thread{std::bind(&CANParser::execute, this, std::placeholders::_1), goal_handle}.detach();
}

