/*
 * @Author: LiFang6606397
 * @Date: 2024-03-26 16:13:48
 * @LastEditors: wei.canming
 * @LastEditTime: 2025-08-04 11:21:33
 * @FilePath: /colcon_ws/src/usbcan/usbcan/src/usbcan_utils.cpp
 * @Description: can通信通用接口抽象
 * 
 * Copyright (c) 2024 by LiFang6606397, All Rights Reserved. 
 */
#include "usbcan_utils.hpp"

unsigned gDevType = 4;
unsigned gDevIdx = 0;
unsigned gChMask = 1;
// 250kbp 
// unsigned gBaud = 0X1C01; 
// 模拟是自收自发
// unsigned gTxType = 2;
// 实车测试
unsigned gTxType = 1;
unsigned gTxSleep = 59;
unsigned gTxFrames = 2;
unsigned gTxCount = 1000;
int gReserved = 0;
// 0X1C01为250kbp ；0X1C03为125kbps
// 堆高车中两条can路中的波特率不一样，在AF06D中两条can路的波特率都是250kbps
std::vector<unsigned> gBaud{0X1C01, 0X1C01};
// unsigned CANIndex = 0;// 使用第几路can


/// @brief 根据传入的通用数据帧结构体填充对应的CAN数据
/// @param can can对象指针
/// @param frame 通用数据帧结构体
void FillFrame(VCI_CAN_OBJ *can, GeneralFrame frame){   
  // 将指定的内存区域设置为一个特定的值,将can内存全设置为0
  memset(can, 0, sizeof(VCI_CAN_OBJ));
  
  // 填充CAN数据
  can->ID = frame.ID;
  can->DataLen = frame.DataLen;
  can->SendType = frame.SendType;
  can->DataLen = frame.DataLen;
  can->ExternFlag = frame.ExternFlag;

  for(int i = 0; i < 8; i++){
    can->Data[i] = frame.can_data[i];
  }
}

/// @brief 定期向EPEC发送数据
/// @param frames 需要发送的通用数据帧结构体
void SendMessages(std::vector<GeneralFrame>& frames){
  // 动态分配内存
  VCI_CAN_OBJ *buff = (VCI_CAN_OBJ *)malloc(sizeof(VCI_CAN_OBJ) * frames.size());
  
  // 错误标识
  bool err = false;

  // 上一次数据发送成功才能继续发送数据
  if(!err){
    // 判断一次需要发送几帧数据，将要发送的数据装载到CAN结构体上
    for(int j = 0; j < frames.size(); j++){
      FillFrame(&buff[j], frames[j]);
    }

    auto start = std::chrono::steady_clock::now(); // 开始计时
    if(frames.size() != VCI_Transmit(gDevType,gDevIdx, 0, &buff[0], frames.size())){
      RCLCPP_INFO_STREAM(
        rclcpp::get_logger("rclcpp"),
        "----CAN" << 0 << "TX failed----" <<
        "ID = 0x" << buff->ID);
      err = true;
    }
    auto end = std::chrono::steady_clock::now(); // 结束计时
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count(); // 计算时间差
    auto remainingTime = 60 - elapsedTime;
    std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime)); // 以每60ms一次的频率发送

    end = std::chrono::steady_clock::now(); // 计算带休眠运行时间
    auto diff = end - start;
    //  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
    //    "Code executed in " << diff.count() * 1e-6 << " ms\n"); 
  } 
  free(buff);
}

GeneralFrame gen_nmt_command(unsigned int command ,unsigned int node_id){
  GeneralFrame send_nmt_command;

  send_nmt_command.ID = 0x00; //设备的节点号
  send_nmt_command.SendType = gTxType;
  send_nmt_command.DataLen = 2; //只用到两位 其余位闲置
  send_nmt_command.ExternFlag = 0;
  // 第一位是命令还是节点编号
  send_nmt_command.can_data[0] = command;
  send_nmt_command.can_data[1] = node_id;
  
  return send_nmt_command;
}

// ------------------------------------------------------------堆高车----------------------------------------------------------------------------------------
/// @return 通用数据帧结构体 GeneralFrame
GeneralFrame gen_0x184(int speed, int angle){
  GeneralFrame send_0x184;

  send_0x184.ID = 0x184;
  send_0x184.SendType = gTxType;
  send_0x184.DataLen = 8;
  send_0x184.ExternFlag = 0;

  send_0x184.can_data[0] = speed & 0xff;
  send_0x184.can_data[1] = speed >> 8;
  send_0x184.can_data[2] = 0;
  send_0x184.can_data[3] = 0;
  send_0x184.can_data[4] = angle & 0xff;
  send_0x184.can_data[5] = angle >> 8;
  send_0x184.can_data[6] = 0;
  send_0x184.can_data[7] = 0;

  return send_0x184;
}

/// @brief 包含AGV控制模式的数据帧
/// @param AGV_ID AGV车辆ID
/// @param OperationMode 操作模式. 0:自动  1:手动 2:半自动 3:半手动
/// @param CurrentOrderMode 当前命令模式. 1:上位命令 2:本地命令 3:带交通管理的本地命令
/// @param CurrentNavMethod 当前导航模式. 1:反光板 2:色带 4:磁条 5: 反光墙 6:自然 7:二维码
/// @param OperationCode 调度系统发送任务类型.  1:取货 2:放货 8:充电
/// @return 通用数据帧结构体 GeneralFrame
GeneralFrame gen_0x204(
  unsigned int AGV_ID,
  unsigned int OperationMode,
  unsigned int CurrentOrderMode,
  unsigned int CurrentNavMethod,
  unsigned int OperationCode){
  
  GeneralFrame send_0x204;

  send_0x204.ID = 0x204;
  send_0x204.SendType = gTxType;
  send_0x204.DataLen = 6;
  send_0x204.ExternFlag = 0;

  send_0x204.can_data[0] = AGV_ID & 0xff;
  send_0x204.can_data[1] = AGV_ID >> 8;
  send_0x204.can_data[2] = OperationMode; // 0
  send_0x204.can_data[3] = CurrentOrderMode;  //0x01
  send_0x204.can_data[4] = CurrentNavMethod;  //0x01
  send_0x204.can_data[5] = OperationCode;

  return send_0x204;
}


GeneralFrame gen_0x304(int vel_command, int ang_command, int fork_height_command, int fork_rotation_command){
  
  GeneralFrame send_0x304;

  send_0x304.ID = 0x304;
  send_0x304.SendType = gTxType;
  send_0x304.DataLen = 8;
  send_0x304.ExternFlag = 0;

  send_0x304.can_data[0] = vel_command & 0xff;
  send_0x304.can_data[1] = vel_command >> 8;
  send_0x304.can_data[2] = ang_command & 0xff;
  send_0x304.can_data[3] = ang_command >> 8;
  send_0x304.can_data[4] = fork_height_command & 0xff;
  send_0x304.can_data[5] = fork_height_command >> 8;
  send_0x304.can_data[6] = fork_rotation_command & 0xff;
  send_0x304.can_data[7] = fork_rotation_command >> 8;
  return send_0x304;
}

/// @return 通用数据帧结构体 GeneralFrame
GeneralFrame gen_0x404(int Enable){

  GeneralFrame send_0x404;

  send_0x404.ID = 0x404;
  send_0x404.SendType = gTxType;
  send_0x404.DataLen = 8;
  send_0x404.ExternFlag = 0;
  
  send_0x404.can_data[0] = Enable & 0xff;;
  send_0x404.can_data[1] = Enable >> 8;
  send_0x404.can_data[2] = 0;
  send_0x404.can_data[3] = 0; 
  send_0x404.can_data[4] = 0;
  send_0x404.can_data[5] = 0;
  send_0x404.can_data[6] = 0;
  send_0x404.can_data[7] = 0;
  
  return send_0x404;
}


GeneralFrame gen_0x264(unsigned int speedScale, unsigned int distanceScale)
{
   GeneralFrame send_0x264;

  send_0x264.ID = 0x264;
  send_0x264.SendType = gTxType;
  send_0x264.DataLen = 8;
  send_0x264.ExternFlag = 0;
  
  send_0x264.can_data[0] = (speedScale >> 24) & 0xFF;
  send_0x264.can_data[1] = (speedScale >> 16) & 0xFF;
  send_0x264.can_data[2] = (speedScale >> 8) & 0xFF;
  send_0x264.can_data[3] = speedScale & 0xFF;

  send_0x264.can_data[4] = (distanceScale >> 24) & 0xFF;
  send_0x264.can_data[5] = (distanceScale >> 16) & 0xFF;
  send_0x264.can_data[6] = (distanceScale >> 8) & 0xFF;
  send_0x264.can_data[7] = distanceScale & 0xFF;

  
  return send_0x264;
}
// ---------------------------------------------------------------------堆高车----------------------------------------------------------------------------




/// @brief 将两个字节转换为一个16位的十进制数。输入：十进制数，表示高八位和低八位
/// @param high 高8位的整数
/// @param low 低8位的整数
/// @return 返回转换后的16位十进制数
int toDecimal(int high, int low) {
    std::string hi, lo;
    int result;
    
    // 十进制转化为二进制
    for(int i = 7; i>=0; i--){
      hi += std::to_string(((high >> i) & 1));
    }
    
    for(int j = 7; j>=0; j--){
      lo += std::to_string(((low >> j) & 1));
    } 

    std::bitset<16> binaryData(hi + lo);
    
    if(binaryData[15]==1){
      binaryData.flip();
      result = std::stoi(binaryData.to_string(), nullptr, 2);
      result = -(result+1); 
    }else{
      result = std::stoi(binaryData.to_string(), nullptr, 2);
    }

    return result;
}


