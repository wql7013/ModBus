#ifndef MOTECMODBUS_H_
#define MOTECMODBUS_H_
/**** ModBus 主/从机协议 ****
** 支持 ASCII / RTU 两种模式
** 可以创建多个实例
** 读写寄存器使用非堵塞式, 通过绑定回调函数获取结果
** 使用方法:
**** 1.主机
****** 调用ModBus_setup配置
****** 在串口接收中断函数中调用ModBus_readByteFromOuter
****** 循环调用ModBus_Master_loop
****** 调用ModBus_getRegister读目标设备寄存器值
****** 调用ModBus_setRegister写目标设备单寄存器
****** 调用ModBus_setRegisters写目标设备多寄存器
**** 2.从机
****** 调用ModBus_setup配置参数
****** 调用ModBus_attachRegisterHandler绑定获取和设置寄存器的函数
****** 在loop中调用ModBus_Slave_loop
**** 3.函数形参看后面对外接口部分
*/

// 使用主机/从机协议, 可同时使用
#define MODBUS_MASTER
//#define MODBUS_SLAVE

#define _UNIT_TEST
//#define DEBUG
//#define _DELAY_DEBUG

#ifdef _UNIT_TEST

#ifndef MODBUS_MASTER
#define MODBUS_MASTER
#endif // !MODBUS_MASTER

#ifndef MODBUS_SLAVE
#define MODBUS_SLAVE
#endif // !MODBUS_SLAVE

#endif // _UNIT_TEST

#ifdef DEBUG
#define MODBUS_DEBUG(x) print_controlled x
#else
#define MODBUS_DEBUG(x)  
#endif // DEBUG

#ifdef _DELAY_DEBUG
#define MODBUS_DELAY_DEBUG(x) print_controlled x
#else
#define MODBUS_DELAY_DEBUG(x)  
#endif // DEBUG

#define MODBUS_REGISTER_LIMIT 6 // 一次最多读写寄存器个数
#define MODBUS_BUFFER_SIZE ((MODBUS_REGISTER_LIMIT)*4+20) // 数据包最大长度(写多个寄存器的数据包长度)
#define MODBUS_WAITFRAME_N 5  // 指令缓存最大个数
#define MODBUS_DEFAULT_BAUD 9600 // 默认数据收发速率, 9600bps

#include <assert.h>
#include <stdint.h>
#include <string.h>
typedef unsigned char byte;
typedef unsigned char u8;
typedef unsigned int u32;

// TODO: 获取毫秒系统时间的函数, 根据具体系统进行定义
int millis();

typedef enum {
	ASCII,
	RTU
} MODBUS_MODE_TYPE;

typedef enum {
	READ_REGISTER = 0x03,
	WRITE_SINGLE_REGISTER = 0x06,
	WRITE_MULTI_REGISTER = 0x10,
} MODBUS_FUNCTION_TYPE;

typedef struct _MODBUS_SETTING_T { // ModBus实例配置信息类型
	uint8_t address; // 目标设备地址
	MODBUS_MODE_TYPE frameType; // 工作模式, 包括 ASCII和RTU
	u32 baudRate; // 数据速率, 比如9600或115200等
	u8 register_access_limit; // 一次最多读/写寄存器个数
	void(*sendHandler)(byte*, size_t); // 用于发送数据的函数, 函数参数(byte* data, size_t size)数据首地址和数据字节数
} ModBus_Setting_T;

typedef struct _MODBUS_FRAME_T {
	u8 index; // 指令序号
	byte data[MODBUS_BUFFER_SIZE + 2]; // 数据, 多分配两字节保证安全
	u8 size; // 数据长度
	MODBUS_FUNCTION_TYPE type; // 指令类型
	u32 time; // 指令开始时间
	void* responseHandler; // 指令执行结束回调函数指针
	u8 responseSize; // 返回帧长度
	uint16_t address; // 访问寄存器的地址
	u8 count; // 访问寄存器的个数
} MODBUS_FRAME_T;

typedef void(*GetReponseHandler_T)(uint16_t*, uint16_t); // 读取寄存器回调函数指针类型, 回到函数参数(寄存器值缓冲区首地址, 寄存器个数)
typedef void(*SetReponseHandler_T)(uint16_t, uint16_t); // 写入寄存器回调函数指针类型, 回调函数参数(寄存器地址, 写入个数)

typedef struct __MODBUS_Parameter {
	uint8_t m_address; // 从机设备地址
	MODBUS_MODE_TYPE m_modeType; // 协议模式: ASCII / RTU
	byte m_receiveFrameBuffer[MODBUS_BUFFER_SIZE + 2]; // 接收数据包, 多分配两字节保证安全
	size_t m_receiveFrameBufferLen;  // 接收到的数据字节数

	volatile byte m_receiveBufferTmp[MODBUS_BUFFER_SIZE + 2]; // 临时储存的接收数据, 由于中断函数会修改此变量, 因而采用循环存取, 避免中断函数外部修改此变量
	volatile byte* m_pBeginReceiveBufferTmp; // 循环存取区开始位置
	volatile byte* m_pEndReceiveBufferTmp; // 循环存取区结束位置的下一个位置
	byte m_hasDetectedBufferStart;

	uint16_t m_registerData[MODBUS_REGISTER_LIMIT + 2]; // 缓存读寄存器数据
	uint16_t m_registerCount;
	u8 m_registerAcessLimit;

	volatile u32 m_lastReceivedTime; // 最近一次接受到字节数据的时刻
	u32 m_lastSentTime; // 最近一次发送数据的时刻
	u32 m_receiveTimeout; // 设定的接收等待下一字符超时时间
	u32 m_sendTimeout; // 设定等待返回帧超时时间

	byte m_faston; // 是否开启快速模式

	void(*m_SendHandler)(byte*, size_t); // 发送数据函数, 用于向外部设备传递数据

#ifdef MODBUS_MASTER // 主机
	MODBUS_FRAME_T m_sendFrames[MODBUS_WAITFRAME_N + 2]; // 发送数据包队列
	size_t m_sendFramesN; // 发送数据包队列长度
	u8 m_nextFrameIndex; // 下一数据包序号
	byte m_waitingResponse; // 正在等待返回帧
#endif // MODBUS_MASTER

#ifdef MODBUS_SLAVE // 从机
	byte m_sendFrameBuffer[MODBUS_BUFFER_SIZE];
	u8 m_sendFrameBufferLen;

	size_t(*m_GetRegisterHandler)(uint16_t, uint16_t, uint16_t*); // 读取寄存器函数, 函数参数(寄存器首地址, 寄存器个数, 读出的数据), 返回成功读取的个数
	size_t(*m_SetRegisterHandler)(uint16_t, uint16_t, uint16_t*); // 设置寄存器函数, 函数参数(寄存器地址, 写入个数, 写入数据), 返回成功设置的个数
#endif // MODBUS_SLAVE


} ModBus_parameter;

/************ 对外接口 BEGIN ***********/
void ModBus_setup(ModBus_parameter* ModBus_para, ModBus_Setting_T setting); // 配置ModBus实例
void ModBus_readByteFromOuter(ModBus_parameter* ModBus_para, byte receivedByte); // 传递字节数据到ModBus协议
void ModBus_fastMode(ModBus_parameter* ModBus_para, byte faston); // 是否开启快速指令模式, 快速模式不缓存指令, 关闭快速模式可保证指令被执行但可能有延迟

/** 设置数据收发速率 **/
/*** 参数 ***
** baud: 数据收发速率
** 注: 可以不设置, 使用默认超时时间( 符合波特率9600, 波特率大于9600可以不设置, 小于必须设置 )
***/
void ModBus_setBitRate(ModBus_parameter* ModBus_para, u32 baud);

/** 设置接收超时时间 **/
/*** 参数 ***
** receiveTimeout: 接收时等待下一字节超时时间, 根据串口速率确定
** sendTimeout: 发送后等待返回帧超时时间, 根据串口速率确定
** 注: 可以不设置, 使用默认超时时间( 符合波特率9600, 波特率大于9600可以不设置, 小于必须设置 )
***/
void ModBus_setTimeout(ModBus_parameter* ModBus_para, u32 receiveTimeout, u32 sendTimeout);


#ifdef MODBUS_MASTER // ModBus主机接口
// 主机loop函数
void ModBus_Master_loop(ModBus_parameter* ModBus_para);

/** 读取寄存器 **/
/*** 参数 ***
** address: 寄存器首地址
** count: 读取寄存器个数
** GetReponseHandler: 读取结果回调函数, 传入参数(uint16_t* buff, uint16_t buffLen),读取未成功传入参数(0,0)
** 返回指令序号(大于0), 以便在回调函数中判断完成的是哪一指令, 不可发送则返回0
***/
byte ModBus_getRegister(ModBus_parameter* ModBus_para, uint16_t address, uint16_t count, void(*GetReponseHandler)(uint16_t*, uint16_t));

/** 写单个寄存器 **/
/*** 参数 ***
** address: 寄存器首地址
** data: 待写入数据
** SetReponseHandler: 写入结果回调函数, 传入参数(uint16_t address, uint16_t count), 参数包括首地址和寄存器个数,写指令超时传入参数(0,0)
** 返回指令序号(大于0), 以便在回调函数中判断完成的是哪一指令, 不可发送则返回0
***/
byte ModBus_setRegister(ModBus_parameter* ModBus_para, uint16_t address, uint16_t data, void(*SetReponseHandler)(uint16_t, uint16_t));

/** 写多个寄存器 **/
/*** 参数 ***
** address: 寄存器首地址
** data: 待写入数据
** count: 待写入寄存器个数
** SetReponseHandler: 写入结果回调函数, 传入参数(uint16_t address, uint16_t count), 参数包括首地址和寄存器个数,写指令超时传入参数(0,0)
** 返回指令序号(大于0), 以便在回调函数中判断完成的是哪一指令, 不可发送则返回0
***/
byte ModBus_setRegisters(ModBus_parameter* ModBus_para, uint16_t address, uint16_t* data, uint16_t count, void(*SetReponseHandler)(uint16_t, uint16_t));

#endif


#ifdef MODBUS_SLAVE // ModBus从机接口
// 从loop函数
void ModBus_Slave_loop(ModBus_parameter* ModBus_para);

// 从机设置读写寄存器函数
void ModBus_attachRegisterHandler(ModBus_parameter* ModBus_para, size_t(*GetRegisterHandler)(uint16_t, uint16_t, uint16_t*), size_t(*SetRegisterHandler)(uint16_t, uint16_t, uint16_t*));

#endif
/**************** 对外接口 END ***************/

#endif
