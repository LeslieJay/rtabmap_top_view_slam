/*
 * @Author: liwenrui
 * @Date: 2024-11-07 09:13:03
 * @LastEditors: wei.canming
 * @LastEditTime: 2025-08-04 15:31:29
 * @FilePath: /colcon_ws/src/usbcan/usbcan/src/keyboard_controller.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by du.xiaoying1 , All Rights Reserved. 
 */



#include "keyboard_controller.h"


KeyboardController::KeyboardController() : Node("keyboard_controller")
{
  // 初始化终端设置
  struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  // 设置非阻塞读取
  fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

  // 初始化速度
  left_speed_ = 0;
  right_speed_ = 0;

  // 创建发布者
  speed_pub_ = this->create_publisher<usbcan_jay_pkg::msg::DifferentialWheel>("ShaoGuan/AG05D/N38/DifferentialWheel_vel", 10);
}

void KeyboardController::spin()
{
  char key;
  while (rclcpp::ok())
  {
    if (read(STDIN_FILENO, &key, 1) > 0)
    {
      handle_key(key);// 处理键盘输入
    }
    publish_speeds();// 发布速度
    rclcpp::spin_some(this->get_node_base_interface());// 处理ROS消息
    usleep(10000);  // 10ms sleep
  }
}


// S曲线计算实时速度
// 该函数实现了S型加速曲线算法，使机器人的速度变化更加平滑
void KeyboardController::cal_curespta(CurveObject* curve)
{
  // 初始化功率和速度变量
  double power = 0.0;
  double speed = 0.0;
  
  // 计算当前时间在加速周期中的相对位置，范围从-1到1
  // 当aTimes从0增长到maxTimes，power从-1变化到1
  power = ( 2 * ( (float)curve->aTimes) - ((float)curve->maxTimes)) / ((float)curve->maxTimes);
  // power = ( (2 * curve->aTimes) - curve->maxTimes) / curve->maxTimes;
  std::cout<<"pow0: "<<power<<std::endl;
  
  // 将power值乘以曲线拉伸系数的负值
  // flexible值越大，曲线越陡峭，加速度变化越明显
  power = (0.0 - curve->flexible) * power;
  
  // 计算S型曲线的值，使用指数函数
  // 这是Sigmoid函数的一部分：1/(1+e^(-x))的变形
  speed = 1 + exp(power);
  
  // 根据起始速度和目标速度，计算当前应有的速度值
  // 这种计算方式可以产生平滑的加速过程
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
// 该函数根据目标速度和当前速度计算平滑的速度变化过程
void KeyboardController::velocity_curve(CurveObject* curve)
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
     temp = std::trunc(temp);// 取整
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

void KeyboardController::forward_accelerate()
{ 
  cure.maxTimes = 0;
  cure.aTimes = 0;
  cure.currentSpeed = 0; 
  cure.startSpeed = 0;
  cure.targetSpeed = 0.5;//单位为0.6m/s
  cure.stepspeed = 0.001; //0.00625 64 /0.002 200
  cure.speedMax = 0.5;
  cure.speedMin = -0.5;

  // 值越大前期变换就越大
  cure.flexible = 12;
 
  int time_count = 0;
 
  // while(rotation_cure.aTimes <= 1000) //1300 840但是由于地面不平 当从高处往低处时会发生一点滑行 导致丢吗 如 84-->91,所以把值改小一点
  //while(cure.aTimes <= 400) //1300 	nav_msgs 
	// tf2_ros 840但是由于地面不平 当从高处往低处时会发生一点滑行 导致丢吗 如 84-->91,所以把值改小一点
  {
    // 暂停一段时间以避免过度占用CPU资源
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    time_count++;
    velocity_curve(&cure);

    // 原地旋转，左右轮速度值一致，符号相反，右旋转，右轮速度为负
    left_speed_ = cure.currentSpeed * 1000;
    right_speed_ = cure.currentSpeed * 1000;

    auto msg = std::make_shared<usbcan_jay_pkg::msg::DifferentialWheel>();
    msg->left_speed = left_speed_;
    msg->right_speed = right_speed_;

    speed_pub_->publish(*msg);
    std::cout << "左右轮速度: " << left_speed_ << " mm/s, " << right_speed_ << " mm/s" << std::endl;
   
  }
}

void KeyboardController::back_accelerate()
{ 
  cure.maxTimes = 0;
  cure.aTimes = 0;
  cure.currentSpeed = 0; 
  cure.startSpeed = 0;
  cure.targetSpeed = -0.3;//单位为0.6m/s
  cure.stepspeed = 0.000333333; //0.00625 64 /0.002 200
  cure.speedMax = 0;
  cure.speedMin = -0.3;

  // 值越大前期变换就越大
  cure.flexible = 15;
 
  int time_count = 0;
 
  // while(rotation_cure.aTimes <= 1000) //1300 840但是由于地面不平 当从高处往低处时会发生一点滑行 导致丢吗 如 84-->91,所以把值改小一点
  while(cure.aTimes <= 899) //1300 840但是由于地面不平 当从高处往低处时会发生一点滑行 导致丢吗 如 84-->91,所以把值改小一点
  {
    time_count++;
    velocity_curve(&cure);

    // 原地旋转，左右轮速度值一致，符号相反，右旋转，右轮速度为负
    left_speed_ = cure.currentSpeed * 1000;
    right_speed_ = cure.currentSpeed * 1000;

    auto msg = std::make_shared<usbcan_jay_pkg::msg::DifferentialWheel>();
    msg->left_speed = left_speed_;
    msg->right_speed = right_speed_;

    speed_pub_->publish(*msg);
    std::cout << "左右轮速度: " << left_speed_ << " mm/s, " << right_speed_ << " mm/s" << std::endl;
    
    }
}

// void KeyboardController::deaccelerate()
// {

// }

void KeyboardController::handle_key(char key)
{
  switch (key)
  {
    case 'z':  // 前进
      left_speed_ = 30 ; // 左轮正向
      right_speed_ = 30;  // 右轮反向
      // 从二维码处开始：右偏24 268 ->-13 266
      // 速度差越小转弯半径越小，转弯越缓；在同样的速度差的情况下速度越大，转弯半径越大
      break;
    case 'c':  // 后退
      left_speed_ = -30;  // 左轮反向
      right_speed_ = -30;  // 右轮正向
      break;
    case 'q':  //highspeed forward
      left_speed_ = 500;  
      right_speed_ = 500;  
      break;
    case 'w':  // 前进d
      // forward_accelerate();
      left_speed_ = 200 ; // 左轮正向
      right_speed_ = 200;  // 右轮反向
      // 从二维码处开始：右偏24 268 ->-13 266
      // 速度差越小转弯半径越小，转弯越缓；在同样的速度差的情况下速度越大，转弯半径越大
      break;
    case 's':  // 后退
      // back_accelerate();
      left_speed_ = -200;  // 左轮反向
      right_speed_ = -200;  // 右轮正向
      break;
    case 'a':  // 左
      left_speed_ = -50;  // 左轮正向
      right_speed_ = 50;  // 右轮反向
      break;
    case 'd':  // 右转
      left_speed_ = 100;  // 左轮反向
      right_speed_ = -100;  // 右轮正向
      break;
    case 'x':  // 停止
      left_speed_ = 0;
      right_speed_ = 0;
      break;
    default:
      return;
  }
}

void KeyboardController::publish_speeds()
{
  auto msg = std::make_shared<usbcan_jay_pkg::msg::DifferentialWheel>();
  msg->left_speed = left_speed_;
  msg->right_speed = right_speed_;

  speed_pub_->publish(*msg);
  std::cout << "左右轮速度: " << left_speed_ << " mm/s, " << right_speed_ << " mm/s" << std::endl;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardController>();
  node->spin();
  rclcpp::shutdown();
  return 0;
}