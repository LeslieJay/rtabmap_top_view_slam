/*
 * @Author: LiFang6606397
 * @Date: 2024-07-15 10:24:4
 * @LastEditors: wei.canming
 * @LastEditTime: 2025-08-21 14:30:34
 * @FilePath: /colcon_ws/src/usbcan/src/main.cpp
 * @Description: 在李芳代码的基础上，修改了里程计计算方法和ros节点的时间策略
 * 
 * Copyright (c) 2024 by LiFang6606397, All Rights Reserved. 
 */
#include "usbcan_parser.hpp"
#include "builtin_interfaces/msg/time.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

double WHEEL_DISTANCE = 0.544; //小车轮间距544.5毫米
double CORRECTION_FACTOR = 1.0;

// pose of base_link
double distance_x = 0;
double distance_y = 0;
double theta_k = 0;

rclcpp::Time last_time_;
std::mutex que_mtx_;
bool ctl_queue_ = true;
#define SAMPLING_INTERVAL 60 // 60ms

/// @brief 读取can0数据的线程
/// @param ctx can通信选项
/// @param node ros2节点，用于发布消息
void rx_thread_0(RX_CTX& ctx, std::shared_ptr<CANParser> node) {  
  ctx.channel = 0; // 指定读取 CAN0
  ctx.stop = 0;
  ctx.error = 0;
  ctx.total = 0; // reset counter
  VCI_CAN_OBJ can[RX_BUFF_SIZE]; // buffer
  

  while(!ctx.stop && !ctx.error){
    int cnt = VCI_Receive(gDevType, gDevIdx, ctx.channel, can, RX_BUFF_SIZE, RX_WAIT_TIME);
    if(!cnt) continue;

    last_time_ = node->get_clock()->now();

    int DIFF = 159888298;//! 和图漾相机的时钟的差值

    for(int i = 0; i < cnt; i++){
      ctx.total += 1;
      // 监听 0x184
      if (can[i].ID == 388) {

        // 左轮线速度 m/s ,can协议中是mm/s ,1e-3进行转换
        double vel_left_encode = toDecimal(can[i].Data[1], can[i].Data[0]) * 1e-3;
        // 右轮线速度
        double vel_right_encode = toDecimal(can[i].Data[3], can[i].Data[2]) * 1e-3;
         
        // 两轮间距的1/2，https://zhuanlan.zhihu.com/p/635908682
        double H = WHEEL_DISTANCE / 2;
        // 两轮中心运动半径 
        double motion_radius = (H * (vel_left_encode + vel_right_encode) ) / (vel_right_encode - vel_left_encode ); //不需要再除以2了，因为H已经是轮间距的一半了
        // 两轮差速轮的角速度，单位为弧度
        double vel_ang_rad; 
        // agv整车速度
        double vel_agv; 

        vel_agv = (vel_left_encode + vel_right_encode)/2;
        
        /**
         * @brief 处理特殊情况
         * 1.直行：角速度为 0
         * 2.静止：角速度和线速度均为 0 
         */

        // left = right 直行 agv_v =left=right
        if(vel_left_encode == vel_right_encode)
        {
          // vel_ang_rad = vel_agv / motion_radius;//w = v /R motion_radius是一个无穷大的值会报错吗
          vel_ang_rad = 0.0;
        }else{
          vel_ang_rad = (vel_right_encode - vel_left_encode ) / WHEEL_DISTANCE;
        }

        if(vel_left_encode ==0 && vel_right_encode == 0)
        {
          vel_agv = 0;
          vel_ang_rad = 0;
        }
        auto current_time = node->get_clock()->now();
        double time_diff_sec = (current_time.seconds() - last_time_.seconds())*1e3;
        RCLCPP_INFO_STREAM(node->get_logger(),
              "last_time_:  " << std::dec << last_time_.seconds() << 
              "\ncurrent_time:  " << std::dec << current_time.seconds() << 
              "\ntime_diff_sec:" << std::dec << time_diff_sec << std::endl);

        last_time_ = current_time;
        // cout<<"time_diff:"<<time_diff<<"delta_theta:"<<delta_theta<<std::endl;
        double delta_theta = vel_ang_rad * time_diff_sec; //delta_theta的单位是弧度
        // ------------- 位姿更新 -------------

        if(std::fabs(vel_ang_rad) < 1e-6){
          // 运动幅度较小时
          // Runge-Kutta 2nd order integration:
          
          double direction = theta_k + 0.5 * delta_theta; // 预测中间角度，然后更新位姿 //!  根据ros2_controllers, 应该是角速度而不是角度增量
          distance_x += vel_agv * cos(direction) * time_diff_sec;
          distance_y += vel_agv * sin(direction) * time_diff_sec;
          theta_k    += delta_theta;
        }else{
          // Exact integration
          double theta_old = theta_k;
          theta_k    +=  delta_theta;
          distance_x +=  motion_radius * (sin(theta_k) - sin(theta_old));
          distance_y += -motion_radius * (cos(theta_k) - cos(theta_old));          
        }



        //? 为什么是取余？可能超过360°，theta_k对2π取余还是自己，theta_k是位姿角
        theta_k = std::fmod(theta_k, 2 * PI);
        

        // 每1秒发布一次位姿信息
        static auto last_log_time = std::chrono::steady_clock::now();
        auto current_log_time = std::chrono::steady_clock::now();
        auto log_duration = std::chrono::duration_cast<std::chrono::seconds>(current_log_time - last_log_time);
        
        if (log_duration.count() >= 1) {
            RCLCPP_INFO_STREAM(node->get_logger(),
              "位姿: x: " << std::dec << distance_x << 
              "\ty:" << std::dec << distance_y << std::endl);
            last_log_time = current_log_time;
        }

        
        nav_msgs::msg::Odometry usbcan;
        usbcan.header.frame_id = "odom";
        usbcan.header.stamp = node->get_clock()->now();

        usbcan.child_frame_id = "base_link";
        // 此处的theta_k已经更新过了，所以减去一半的增量才会得到正确的结果
        usbcan.twist.twist.linear.x = vel_agv * cos(theta_k - 0.5 * delta_theta);
        usbcan.twist.twist.linear.y = vel_agv * sin(theta_k - 0.5 * delta_theta);
        usbcan.twist.twist.linear.z = 0;

        usbcan.twist.twist.angular.x = 0;
        usbcan.twist.twist.angular.y = 0;
        usbcan.twist.twist.angular.z = vel_ang_rad;
        
        usbcan.pose.pose.orientation.x = 0;
        usbcan.pose.pose.orientation.y = 0;
        usbcan.pose.pose.orientation.z = sin(theta_k*0.5);
        usbcan.pose.pose.orientation.w = cos(theta_k*0.5);

        usbcan.pose.pose.position.x = 1.45 * distance_x;
        usbcan.pose.pose.position.y = 1.45 * distance_y;
        usbcan.pose.pose.position.z = 0;
        
        node->getPub()->publish(usbcan);

        RCLCPP_INFO_STREAM(node->get_logger(),
              "位姿: x: " << std::dec << distance_x << 
              "\ty:" << std::dec << distance_y << std::endl);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = usbcan.header.stamp;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        t.transform.translation.x = 1.45 * distance_x;
        t.transform.translation.y = 1.45 * distance_y;
        t.transform.translation.z = 0.0;

        t.transform.rotation.x = 0;
        t.transform.rotation.y = 0;
        t.transform.rotation.z = sin(theta_k*0.5);
        t.transform.rotation.w = cos(theta_k*0.5);
        
        node->getTF()->sendTransform(t);

        RCLCPP_INFO_STREAM(node->get_logger(),
              "TF: x: " << std::dec << distance_x << 
              "\ty:" << std::dec << distance_y << std::endl);
      }
    
    }
  }
}

/// @brief 循环处理队列并发送CAN数据帧
/// @param node ros2节点，用于处理队列、发布日志
void process_cmd_queue(std::shared_ptr<CANParser> node){
  
  // RCLCPP_INFO_STREAM(node->get_logger(), "processing can command queue.");
  
  std::vector<GeneralFrame> frames;
  while (ctl_queue_){
    frames.clear();
    if(!node->getQ()->empty()){
      {
        std::lock_guard<std::mutex> lock(que_mtx_);
        while(!node->getQ()->empty()){
          frames.emplace_back(node->getQ()->front());
          node->getQ()->pop();
        }
      }
    }
    
    // RCLCPP_INFO_STREAM(node->get_logger(), "frames filled size " << frames.size());
    
    if(!frames.empty()){
      SendMessages(frames);
    }

    // 以usbcan的控制周期为单位进行休眠
    std::this_thread::sleep_for(std::chrono::milliseconds(SAMPLING_INTERVAL));
  }
}

/// @brief 初始化usbcan
/// @return 是否初始化成功
bool usbcan_init(){
  
  // 打开设备
  if(!VCI_OpenDevice(gDevType, gDevIdx, gReserved)){
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("main"), "VCI_OpenDevice failed");
    return false;
  }
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("main"), "VCI_OpenDevice done!");
  
  // 初始化各通道配置并启动通道
  for(int CANInd = 0; CANInd < MAX_CHANNELS; CANInd++){
    VCI_INIT_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0xffffffff;
    config.Filter = 1;
    config.Mode = 0;
    config.Timing0 = gBaud[CANInd] & 0xff;
    config.Timing1 = gBaud[CANInd] >> 8;

    if(!VCI_InitCAN(gDevType, gDevIdx, CANInd, &config)){
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger("main"),
        "VCI_InitCAN -> " << std::dec << CANInd << " failed!");
      return false;
    }

    if(!VCI_StartCAN(gDevType, gDevIdx, CANInd)){
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger("main"),
        "VCI_StartCAN -> " << std::dec << CANInd << " failed!");
      return false;
    }
  }
  return true;
}

/// @brief 打印变量及其十六进制表示
/// @param name 变量名
/// @param value 变量值
template<typename T>
void printVariableWithHex(const std::string& name, const T& value) {
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("main"),
    name << " = " << value << "\t in hex: " << std::hex << std::showbase << value);
}

/* ------------------------------------------------------------ */

int main(int argc, char * argv[]){

  rclcpp::init(argc, argv);

  auto parser = std::make_shared<CANParser>();

  printVariableWithHex("DevType", gDevType);
  printVariableWithHex("DevIdx",  gDevIdx);
  printVariableWithHex("TxType",  gTxType);
  printVariableWithHex("TxSleep", gTxSleep);
  printVariableWithHex("ChMask",  gChMask);
  printVariableWithHex("TxFrames",gTxFrames);
  printVariableWithHex("TxCount", gTxCount);
  for (size_t i = 0; i < gBaud.size(); ++i) {
    printVariableWithHex("Baud " + std::to_string(i), gBaud[i]);
  }

  if(!usbcan_init()){
    RCLCPP_INFO_STREAM(rclcpp::get_logger("main"), "Initialize CAN failed!");
    return 0;
  }

  // 创建接收线程
  RX_CTX rx_ctx_0;
  std::thread read_can_0(rx_thread_0, std::ref(rx_ctx_0), parser);


  // 创建发送线程
  std::thread send_can(process_cmd_queue, parser); 

  rclcpp::spin(parser);

  ctl_queue_ = false;
  rx_ctx_0.stop = 1;  // 设置标志以停止线程
  read_can_0.join();  // 等待线程read_can_0结束
  send_can.join();  // 等待线程send_can结束

  VCI_CloseDevice(gDevType, gDevIdx);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("main"), "VCI_CloseDevice!");

  rclcpp::shutdown();
  return 0;
}