#ifndef CONTROLCAN_H
#define CONTROLCAN_H

//接口卡类型定义
#define VCI_PCI5121  1
#define VCI_PCI9810  2
#define VCI_USBCAN1  3
#define VCI_USBCAN2  4
#define VCI_PCI9820  5
#define VCI_CAN232   6
#define VCI_PCI5110  7
#define VCI_CANLite  8
#define VCI_ISA9620  9
#define VCI_ISA5420  10

//CAN错误码
#define  ERR_CAN_OVERFLOW  0x0001  //CAN控制器内部FIFO溢出
#define  ERR_CAN_ERRALARM  0x0002  //CAN控制器错误报警
#define  ERR_CAN_PASSIVE   0x0004  //CAN控制器消极错误
#define  ERR_CAN_LOSE      0x0008  //CAN控制器仲裁丢失
#define  ERR_CAN_BUSERR    0x0010  //CAN控制器总线错误

//通用错误码
#define  ERR_DEVICEOPENED    0x0100  //设备已经打开
#define  ERR_DEVICEOPEN      0x0200  //打开设备错误
#define  ERR_DEVICENOTOPEN   0x0400  //设备没有打开
#define  ERR_BUFFEROVERFLOW  0x0800  //缓冲区溢出
#define  ERR_DEVICENOTEXIST  0x1000  //此设备不存在
#define  ERR_LOADKERNELDLL   0x2000  //装载动态库失败
#define ERR_CMDFAILED        0x4000  //执行命令失败错误码
#define  ERR_BUFFERCREATE    0x8000  //内存不足


//函数调用返回状态值
#define STATUS_OK   1
#define STATUS_ERR  0

#define USHORT unsigned short int
#define BYTE unsigned char
#define CHAR char
#define UCHAR unsigned char
#define UINT unsigned int
#define DWORD unsigned int
#define PVOID void*
#define ULONG unsigned int
#define INT int
#define UINT32 UINT
#define LPVOID void*
#define BOOL BYTE
#define TRUE  1
#define FALSE 0


//1.ZLGCAN系列接口卡信息的数据类型。
typedef  struct  _VCI_BOARD_INFO{
  USHORT  hw_Version;
  USHORT  fw_Version;
  USHORT  dr_Version;
  USHORT  in_Version;
  USHORT  irq_Num;
  BYTE    can_Num;
  CHAR    str_Serial_Num[20];
  CHAR    str_hw_Type[40];
  USHORT  Reserved[4];
} VCI_BOARD_INFO,*PVCI_BOARD_INFO; 

//2.定义CAN信息帧的数据类型。
typedef  struct  _VCI_CAN_OBJ{
  UINT  ID;
  UINT  TimeStamp;
  BYTE  TimeFlag;
  BYTE  SendType;
  BYTE  RemoteFlag; //是否是远程帧
  BYTE  ExternFlag; //是否是扩展帧
  BYTE  DataLen;
  BYTE  Data[8];
  BYTE  Reserved[3];
}VCI_CAN_OBJ,*PVCI_CAN_OBJ;

//3.定义CAN控制器状态的数据类型。
typedef struct _VCI_CAN_STATUS{
  UCHAR  ErrInterrupt;
  UCHAR  regMode;
  UCHAR  regStatus;
  UCHAR  regALCapture;
  UCHAR  regECCapture; 
  UCHAR  regEWLimit;
  UCHAR  regRECounter; 
  UCHAR  regTECounter;
  DWORD  Reserved;
}VCI_CAN_STATUS,*PVCI_CAN_STATUS;

//4.定义错误信息的数据类型。
typedef struct _ERR_INFO{
  UINT  ErrCode;
  BYTE  Passive_ErrData[3];
  BYTE  ArLost_ErrData;
} VCI_ERR_INFO,*PVCI_ERR_INFO;

//5.定义初始化CAN的数据类型
typedef struct _INIT_CONFIG{
  DWORD  AccCode;
  DWORD  AccMask;
  DWORD  Reserved;
  UCHAR  Filter;  // 滤波方式. 1 单滤波; 0 双滤波
  UCHAR  Timing0; // 波特率定时器 0
  UCHAR  Timing1; // 波特率定时器 1
  UCHAR  Mode;    // 模式. 0 读写总线; 1 只读总线
}VCI_INIT_CONFIG,*PVCI_INIT_CONFIG;

/**检查是否在C++编译器下编译。
 * 如果是，那么`EXTERN_C`会被定义为extern "C"，
 * 这样在需要链接到C语言代码的地方使用`EXTERN_C`
 * 就可以确保链接的是正确的、未改编的C函数。
*/
#ifdef __cplusplus
#define EXTERN_C  extern "C"
#else
#define EXTERN_C
#endif

// 打开设备. 一个设备只能打开一次
EXTERN_C DWORD VCI_OpenDevice(DWORD DeviceType,DWORD DeviceInd,DWORD Reserved);
// 关闭设备
EXTERN_C DWORD VCI_CloseDevice(DWORD DeviceType,DWORD DeviceInd);
// 初始化指定的 CAN 通道
EXTERN_C DWORD VCI_InitCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_INIT_CONFIG pInitConfig);


// 获取设备信息
EXTERN_C DWORD VCI_ReadBoardInfo(DWORD DeviceType,DWORD DeviceInd,PVCI_BOARD_INFO pInfo);
// 获取 CAN 卡发生的最近一次错误信息
EXTERN_C DWORD VCI_ReadErrInfo(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_ERR_INFO pErrInfo);
// 获取 CAN 状态
EXTERN_C DWORD VCI_ReadCANStatus(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_STATUS pCANStatus);


// 读/写 设备参数
EXTERN_C DWORD VCI_GetReference(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,DWORD RefType,PVOID pData);
EXTERN_C DWORD VCI_SetReference(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,DWORD RefType,PVOID pData);


// 获取指定 CAN 通道的接收缓冲区中，接收到但尚未被读取的帧数量
EXTERN_C ULONG VCI_GetReceiveNum(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);
// 清空指定 CAN 通道的缓冲区
EXTERN_C DWORD VCI_ClearBuffer(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);


// 启动 CAN 卡的某一个通道，可重复调用
EXTERN_C DWORD VCI_StartCAN(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);
// 复位 CAN 卡的某一个通道，可重复调用
EXTERN_C DWORD VCI_ResetCAN(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd);


// 发送函数，返回值为实际发送成功的帧数
EXTERN_C ULONG VCI_Transmit(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_OBJ pSend,UINT Len);
// 从指定的设备 CAN 通道的接收缓冲区中读取数据
EXTERN_C ULONG VCI_Receive(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_OBJ pReceive,UINT Len,INT WaitTime);

#endif

