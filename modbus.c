#include "modbus.h"
#include <stdarg.h>

/** 配置ModBus实例 **/
/*** 参数 ***
** address: 设备地址
** frameType: 协议模式
** sendHandler: 发送数据的外部接口, 比如绑定到串口发送函数, 传入参数(byte* buff, size_t buffLen), 参数包括数据指针和数据长度
***/
void ModBus_setup( ModBus_parameter* ModBus_para, ModBus_Setting_T setting)
{
	ModBus_para->m_address = setting.address;
	ModBus_para->m_modeType = setting.frameType;
	ModBus_para->m_receiveFrameBufferLen = 0;
	ModBus_para->m_sendFramesN = 0;
	ModBus_para->m_nextFrameIndex = 1; // 数据包序号从1开始

	//ModBus_para->m_receiveBufferTmpLen = 0;
	ModBus_para->m_pBeginReceiveBufferTmp = ModBus_para->m_receiveBufferTmp;
	ModBus_para->m_pEndReceiveBufferTmp = ModBus_para->m_receiveBufferTmp;
	ModBus_para->m_hasDetectedBufferStart = 0;

	ModBus_para->m_registerCount = 0;
	if (setting.register_access_limit > 0 && setting.register_access_limit <= MODBUS_REGISTER_LIMIT)
	{
		ModBus_para->m_registerAcessLimit = setting.register_access_limit;
	}
	else
	{
		ModBus_para->m_registerAcessLimit = MODBUS_REGISTER_LIMIT;
	}

	if (setting.baudRate == 0)
	{
		setting.baudRate = MODBUS_DEFAULT_BAUD;
	}
	ModBus_para->m_receiveTimeout = 4000u * 8u / setting.baudRate + 2u;
	ModBus_para->m_sendTimeout = ((ModBus_para->m_registerAcessLimit * 4u + 20u) * 2000u + 7000u) * 8u / setting.baudRate + 5u;

	ModBus_para->m_lastReceivedTime = ModBus_para->m_lastSentTime = millis();

	ModBus_para->m_faston = 0; // 默认关闭快速模式, 保证初始化的时候指令能按顺序被执行

	ModBus_para->m_SendHandler = setting.sendHandler;

#ifdef MODBUS_MASTER // 主机
	
#endif

#ifdef MODBUS_SLAVE // 从机
	ModBus_para->m_GetRegisterHandler = NULL;
	ModBus_para->m_SetRegisterHandler = NULL;
#endif

}

/** 设置数据收发速率 **/
/*** 参数 ***
** baud: 数据收发速率
** 注: 可以不设置, 使用默认超时时间( 符合波特率9600, 波特率大于9600可以不设置, 小于必须设置 )
***/
void ModBus_setBitRate(ModBus_parameter* ModBus_para, u32 baud)
{
	if (baud > 0)
	{
		ModBus_para->m_receiveTimeout = 4000u * 8u / baud + 2u;
		ModBus_para->m_sendTimeout = ((ModBus_para->m_registerAcessLimit * 4u + 20u) * 2000u + 7000u) * 8u / baud + 15u;
	}
}

/** 设置接收超时时间 **/
/*** 参数 ***
** receiveTimeout: 接收时等待下一字节超时时间, 根据串口速率确定
** sendTimeout: 发送后等待返回帧超时时间, 根据串口速率确定
** 注: 可以不设置, 使用默认超时时间
***/
void ModBus_setTimeout(ModBus_parameter* ModBus_para, u32 receiveTimeout, u32 sendTimeout)
{
	if (receiveTimeout > 0)
		ModBus_para->m_receiveTimeout = receiveTimeout;
	if (sendTimeout > 0)
		ModBus_para->m_sendTimeout = sendTimeout;
}

// RTU模式时, 产生CRC校验码并添加到数据尾部
// Calculate CRC for outcoming buffer
// and place it to end.
// return total length
static size_t GenCRC16(byte* buff, size_t len)
{
	uint16_t crc = 0xFFFF;
	uint16_t pos = 0;
	uint8_t i = 0;
	uint8_t lo = 0;
	uint8_t hi = 0;

	for (pos = 0; pos < len; pos++)
	{
		crc ^= buff[pos];

		for (i = 8; i != 0; i--)
		{
			if ((crc & 0x0001) != 0)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
				crc >>= 1;
		}
	}
	lo = crc & 0xFF;
	hi = (crc >> 8) & 0xFF;

	buff[len++] = lo;
	buff[len++] = hi;
	return len;
}

// RTU模式时, 计算CRC校验码, 给定初值
// Calculate CRC fro incoming buffer
// Return 1 - if CRC is correct, overwise return 0
static byte CheckCRC16(byte* buff, size_t len)
{
	uint16_t crc = 0xFFFF;
	uint16_t pos = 0;
	uint8_t i = 0;
	uint8_t lo = 0;
	uint8_t hi = 0;

	for (pos = 0; pos < len - 2; pos++)
	{
		crc ^= buff[pos];

		for (i = 8; i != 0; i--)
		{
			if ((crc & 0x0001) != 0)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
				crc >>= 1;
		}
	}
	lo = crc & 0xFF;
	hi = (crc >> 8) & 0xFF;
	if ((buff[len - 2] == lo) &&
		(buff[len - 1] == hi))
	{
		return 1;
	}
#ifdef _UNIT_TEST
	printf("CRC Check ERROR\n");
#endif // _UNIT_TEST
	return 0;
}

// ASCII模式时, 产生LRC校验码并添加到数据尾部
static size_t GenLRC(byte* buff, size_t len)
{
	size_t i = len;
	uint8_t uchLRC = 0; /* LRC 字节初始化 */
	while (i--)
		uchLRC += *buff++; /* 累加*/

	uchLRC = ((uint8_t)(-((char)uchLRC)));
	*buff = uchLRC;
	return len + 1;
}

// ASCII模式时, 校验LRC校验码,
static uint8_t CheckLRC(byte* buff, size_t len)
{
	uint8_t uchLRC = 0; /* LRC 字节初始化 */
	uint8_t LRC1 = buff[--len];
	while (len--)
		uchLRC += *buff++; /* 累加*/

	uchLRC = ((uint8_t)(-((char)uchLRC)));
	if (LRC1 == uchLRC)
		return 1;
#ifdef _UNIT_TEST
	printf("LRC Check ERROR\n");
#endif // _UNIT_TEST
	return 0;
}

// ASCII模式时, 接收到的字符串转换为字节二进制
static size_t char2bin(byte* buff, size_t len)
{
	size_t binInd = 0;
	for (size_t i = 0; i < len; i++)
	{
		byte bin = 0, chr = buff[i];
		if ((chr >= '0') && (chr <= '9'))
		{
			bin = (byte)(chr - '0');
		}
		else if ((chr >= 'A') && (chr <= 'F'))
		{
			bin = (byte)(chr - 'A' + 0x0A);
		}
		if (i % 2 == 0)
		{
			buff[binInd] = bin;
		}
		else
		{
			buff[binInd] = (buff[binInd] << 4) + bin;
			binInd++;
		}
	}
	return binInd;
}

// ASCII模式时, 字节二进制转换为字符串, 用于发送
static size_t bin2char_s(byte* buff, size_t len, size_t maxLen)
{
	size_t binLen = len * 2;
	size_t ret = binLen;
	if (ret > maxLen)
	{
		return 0;
	}
	while (len--)
	{
		for (int i = 0; i < 2; i++)
		{
			byte bin = (buff[len] >> 4 * i) & 0x0F;
			char chr = 0;
			if ((bin >= 0) && (bin <= 9))
			{
				chr = (char)(bin + '0');
			}
			else if ((bin >= 0x0A) && (bin <= 0x0F))
			{
				chr = (char)(bin - 0x0A + 'A');
			}
			buff[--binLen] = chr;
		}
	}
	return ret;
}

static MODBUS_FRAME_T* addFrame(ModBus_parameter* ModBus_para)
{
	MODBUS_FRAME_T* pFrame;
	if (ModBus_para->m_sendFramesN >= MODBUS_WAITFRAME_N)
	{
		memcpy(ModBus_para->m_sendFrames, ModBus_para->m_sendFrames + 1, (ModBus_para->m_sendFramesN - 1) * sizeof(MODBUS_FRAME_T));
		pFrame = ModBus_para->m_sendFrames + (ModBus_para->m_sendFramesN - 1);
	}
	else
	{
		pFrame = ModBus_para->m_sendFrames + (ModBus_para->m_sendFramesN++);
	}
	pFrame->index = ModBus_para->m_nextFrameIndex++;
	if (ModBus_para->m_nextFrameIndex == 0) // 指令序号不为0
	{
		ModBus_para->m_nextFrameIndex = 1;
	}
	pFrame->size = 0;
	pFrame->responseHandler = NULL;
	pFrame->time = millis();
	MODBUS_DELAY_DEBUG(("Frame Len %d\n", ModBus_para->m_sendFramesN));
	return pFrame;
}


// 接收字节数据到ModBus协议, 一般在中断函数中调用(如串口接收中断)
void ModBus_readByteFromOuter(ModBus_parameter* ModBus_para, byte receivedByte)
{
#ifdef _UNIT_TEST
	printf("address %02x read byte: %02x\n", ModBus_para->m_address, receivedByte);
#endif // _UNIT_TEST

	/*** 此函数 内部 不可更改 ModBus_para->m_pBeginReceiveBufferTmp 值!!!!!!!!!!!!!
	**** 此函数 外部 不可更改 ModBus_para->m_pEndReceiveBufferTmp 值!!!!!!!!!!!!!!!
	**** 避免内存写冲突, 保证数据完整性***/
	*ModBus_para->m_pEndReceiveBufferTmp = receivedByte;
	if (ModBus_para->m_pEndReceiveBufferTmp >= ModBus_para->m_receiveBufferTmp + MODBUS_BUFFER_SIZE - 1)
	{
		ModBus_para->m_pEndReceiveBufferTmp = ModBus_para->m_receiveBufferTmp;
	}
	else
	{
		ModBus_para->m_pEndReceiveBufferTmp++;
	}
	if (ModBus_para->m_pEndReceiveBufferTmp == ModBus_para->m_pBeginReceiveBufferTmp)
	{
		ModBus_para->m_pEndReceiveBufferTmp--;
		if (ModBus_para->m_pEndReceiveBufferTmp < ModBus_para->m_receiveBufferTmp)
		{
			ModBus_para->m_pEndReceiveBufferTmp = ModBus_para->m_receiveBufferTmp + (MODBUS_BUFFER_SIZE - 1);
		}
	}
	ModBus_para->m_lastReceivedTime = millis();
}

void ModBus_fastMode(ModBus_parameter* ModBus_para, byte faston)
{
	ModBus_para->m_faston = faston;
}


// 检查接收数据包, 存在有效数据返回1, 否则返回0
static byte ModBus_detectFrame(ModBus_parameter* ModBus_para, size_t* restSize)
{
	size_t i = 0, j = 0;
	byte* pEnd, *pBegin;
	size_t lenBufferTmp;
	u8 frameSize = 0;

#ifdef MODBUS_MASTER
	if (ModBus_para->m_sendFramesN > 0)
	{
		frameSize = ModBus_para->m_sendFrames[0].responseSize;
	}
#endif

	pEnd = ModBus_para->m_pEndReceiveBufferTmp;
	pBegin = ModBus_para->m_pBeginReceiveBufferTmp; // volatile变量必须赋值给非volatile变量再操作, 否则有一定几率出现数据不完整
	lenBufferTmp = pEnd - pBegin;
	if (pEnd < pBegin)
	{
		lenBufferTmp = (size_t)MODBUS_BUFFER_SIZE - (pBegin - pEnd);
	}
	*restSize = 0;

	switch (ModBus_para->m_modeType)
	{
	case ASCII:
	{
		if (lenBufferTmp == 0)
		{
			return 0;
		}
		if (!ModBus_para->m_hasDetectedBufferStart)
		{// 检测起始字符
			for (i = 0; i < lenBufferTmp; i++, pBegin++)
			{
				if (pBegin >= ModBus_para->m_receiveBufferTmp + MODBUS_BUFFER_SIZE)
				{
					pBegin = ModBus_para->m_receiveBufferTmp;
				}
				if (*pBegin == ':') // 检测到起始字符
				{
					ModBus_para->m_hasDetectedBufferStart = 1;
					i++;
					pBegin++;
					if (pBegin >= ModBus_para->m_receiveBufferTmp + MODBUS_BUFFER_SIZE)
					{
						pBegin = ModBus_para->m_receiveBufferTmp;
					}
					break;
				}
			}
		}
		if (ModBus_para->m_hasDetectedBufferStart)
		{// 检测结束字符
			for (j = i; j < lenBufferTmp; j++, pBegin++)
			{
				if (pBegin >= ModBus_para->m_receiveBufferTmp + MODBUS_BUFFER_SIZE)
				{
					pBegin = ModBus_para->m_receiveBufferTmp;
				}
				if (*pBegin == '\r') // 检测到结束字符
				{
					break;
				}
				ModBus_para->m_receiveFrameBuffer[ModBus_para->m_receiveFrameBufferLen++] = *pBegin;
			}
		}
		else // 没有检测到起始字符, 则接收数据异常
		{
			ModBus_para->m_pBeginReceiveBufferTmp = pEnd; // 抛弃pBegin到pEnd之间的数据, 保留pEnd之后新加的数据, 因为此上面处理过程中, 可能有新的数据被接收
			ModBus_para->m_receiveFrameBufferLen = 0;
			return 0;
		}
		if (j == lenBufferTmp) // 如果没有检测到结束字符, 返回继续接收
		{
			ModBus_para->m_pBeginReceiveBufferTmp = pEnd;
			return 0;
		}
		else if (j + 1 == lenBufferTmp) // 如果回车结束字符是最后一个字符, 则保留回车字符返回继续接收
		{
			ModBus_para->m_pBeginReceiveBufferTmp = pBegin;;
			return 0;
		}
		pBegin++;
		if (pBegin >= ModBus_para->m_receiveBufferTmp + MODBUS_BUFFER_SIZE)
		{
			pBegin = ModBus_para->m_receiveBufferTmp;
		}
		if (*pBegin != '\n') // 如果下一个字符不是换行符, 则接收数据异常
		{
			ModBus_para->m_hasDetectedBufferStart = 0;
			ModBus_para->m_pBeginReceiveBufferTmp = pEnd;
			ModBus_para->m_receiveFrameBufferLen = 0;
			return 0;
		}

		ModBus_para->m_pBeginReceiveBufferTmp = pEnd;
		ModBus_para->m_receiveFrameBufferLen = char2bin(ModBus_para->m_receiveFrameBuffer, ModBus_para->m_receiveFrameBufferLen);
		if (ModBus_para->m_receiveFrameBuffer[0] != ModBus_para->m_address)
		{
			ModBus_para->m_hasDetectedBufferStart = 0;
			ModBus_para->m_receiveFrameBufferLen = 0;
			return 0;
		}
		if (!CheckLRC(ModBus_para->m_receiveFrameBuffer, ModBus_para->m_receiveFrameBufferLen)) // 如果校验不通过
		{
			ModBus_para->m_hasDetectedBufferStart = 0;
			ModBus_para->m_receiveFrameBufferLen = 0;
			return 0;
		}

		//MODBUS_DEBUG(("ModBus rec: %*s\n", ModBus_para->m_receiveFrameBufferLen, ModBus_para->m_receiveFrameBuffer));

		ModBus_para->m_receiveFrameBufferLen--; // 去除校验码
		ModBus_para->m_hasDetectedBufferStart = 0;

		break;
	}
	case RTU:
	{
		byte isTimeout = 0;
		if (lenBufferTmp == 0) // 由于接收超时, 没有接收到数据
		{
			isTimeout = 1;
		}
		if (!ModBus_para->m_hasDetectedBufferStart)
		{// 检测起始字节
			for (i = 0; i < lenBufferTmp; i++, pBegin++)
			{
				if (*pBegin == ModBus_para->m_address) // 检测到地址
				{
					ModBus_para->m_hasDetectedBufferStart = 1;
					ModBus_para->m_receiveFrameBuffer[ModBus_para->m_receiveFrameBufferLen++] = *pBegin;
					i++;
					pBegin++;
					break;
				}
			}
		}
		if (ModBus_para->m_hasDetectedBufferStart)
		{
			// 拷贝所有临时缓冲区的数据到接收数据缓冲区
			size_t newSize = lenBufferTmp - i;
			if (ModBus_para->m_receiveFrameBufferLen + newSize > MODBUS_BUFFER_SIZE)
			{
				newSize = MODBUS_BUFFER_SIZE - ModBus_para->m_receiveFrameBufferLen;
			}
			if (pBegin <= pEnd)
			{
				memcpy(ModBus_para->m_receiveFrameBuffer + ModBus_para->m_receiveFrameBufferLen, pBegin, newSize);
			}
			else
			{
				memcpy(ModBus_para->m_receiveFrameBuffer + ModBus_para->m_receiveFrameBufferLen, pBegin, (size_t)MODBUS_BUFFER_SIZE - (pBegin - ModBus_para->m_receiveBufferTmp));
				memcpy(ModBus_para->m_receiveFrameBuffer + ModBus_para->m_receiveFrameBufferLen, (void*)ModBus_para->m_receiveBufferTmp, pEnd - ModBus_para->m_receiveBufferTmp);
			}
			ModBus_para->m_receiveFrameBufferLen += newSize;
			ModBus_para->m_pBeginReceiveBufferTmp = pEnd;
		}
		else // 没有检测到起始字符, 则接收数据异常
		{
			ModBus_para->m_pBeginReceiveBufferTmp = pEnd;
			ModBus_para->m_receiveFrameBufferLen = 0;
			return 0;
		}
		if (!(isTimeout // 接收超时
			|| frameSize > 0 && ModBus_para->m_receiveFrameBufferLen >= frameSize // 数据包足够
			|| ModBus_para->m_receiveFrameBufferLen >= MODBUS_BUFFER_SIZE)) // 缓冲区满
		{
			// 接收未结束, 返回继续接收数据
			return 0;
		}
		if (ModBus_para->m_receiveFrameBufferLen < 2) // 接收超时且数据不足为异常
		{
			ModBus_para->m_pBeginReceiveBufferTmp = pEnd;
			ModBus_para->m_receiveFrameBufferLen = 0;
			return 0;
		}
		if (!CheckCRC16(ModBus_para->m_receiveFrameBuffer, ModBus_para->m_receiveFrameBufferLen)) // 如果校验不通过
		{
			if (frameSize > 0 && frameSize < ModBus_para->m_receiveFrameBufferLen)  // 如果数据长度比m_responseFrameLen长, 则尝试以m_responseFrameLen长度接收
			{	
				if (!CheckCRC16(ModBus_para->m_receiveFrameBuffer, frameSize)) // 如果校验不通过, 不为超时或缓冲区满则返回继续接收
				{
					if (isTimeout || ModBus_para->m_receiveFrameBufferLen >= MODBUS_BUFFER_SIZE)
						ModBus_para->m_receiveFrameBufferLen = 0;
					return 0;
				}

				*restSize = ModBus_para->m_receiveFrameBufferLen - frameSize;// 待保留的数据长度
				ModBus_para->m_receiveFrameBufferLen = frameSize;
			}
			else
			{
				ModBus_para->m_receiveFrameBufferLen = 0;
				return 0;
			}
		}
		ModBus_para->m_receiveFrameBufferLen--; // 去除校验码
		ModBus_para->m_hasDetectedBufferStart = 0;
		break;
	}
	default:
		ModBus_para->m_receiveFrameBufferLen = 0;
		return 0;
		break;
	}

	return 1;
}


#ifdef MODBUS_MASTER
/** 读取寄存器 **/
/*** 参数 ***
** address: 寄存器首地址
** count: 读取寄存器个数
** GetReponseHandler: 读取结果回调函数, 传入参数(uint16_t* buff, uint16_t buffLen)
** 返回指令序号(大于0), 以便在回调函数中判断完成的是哪一指令, 不能发送返回0
***/
byte ModBus_getRegister(ModBus_parameter* ModBus_para, uint16_t address, uint16_t count, void(*GetReponseHandler)(uint16_t*, uint16_t))
{
	MODBUS_FRAME_T* pFrame = addFrame(ModBus_para);
	pFrame->type = READ_REGISTER;
	pFrame->responseSize = 0;
	pFrame->responseHandler = GetReponseHandler;
	pFrame->address = address;
	pFrame->count = count;
	if (ModBus_para->m_modeType == ASCII)
	{
		pFrame->data[pFrame->size++] = ':';
	}
	pFrame->data[pFrame->size++] = ModBus_para->m_address; // 设备地址
	pFrame->data[pFrame->size++] = READ_REGISTER; // 功能码, 读寄存器
	pFrame->data[pFrame->size++] = (address >> 8) & 0x0FF; // 寄存器首地址高位
	pFrame->data[pFrame->size++] = address & 0x0FF; // 寄存器首地址低位
	pFrame->data[pFrame->size++] = (count >> 8) & 0x0FF; // 读寄存器个数高位
	pFrame->data[pFrame->size++] = count & 0x0FF; // 读寄存器个数低位
	switch (ModBus_para->m_modeType)
	{
	case ASCII:
		pFrame->size = GenLRC(pFrame->data + 1, pFrame->size - 1) + 1; // 不包括起始字符
		pFrame->size = bin2char_s(pFrame->data + 1, pFrame->size - 1, MODBUS_BUFFER_SIZE) + 1;
		pFrame->data[pFrame->size++] = '\r'; // 结束字符
		pFrame->data[pFrame->size++] = '\n'; // 结束字符
		pFrame->responseSize = 11 + 4 * count; // 返回帧需要的字节数
		break;
	case RTU:
		pFrame->size = GenCRC16(pFrame->data, pFrame->size);
		pFrame->responseSize = 5 + 2 * count; // 返回帧需要的字节数
		break;
	default:
		break;
	}

	return pFrame->index;
}

/** 写单个寄存器 **/
/*** 参数 ***
** address: 寄存器首地址
** data: 待写入数据
** SetReponseHandler: 写入结果回调函数, 传入参数(uint16_t address, uint16_t count), 参数包括首地址和寄存器个数
** 返回指令序号, 以便在回调函数中判断完成的是哪一指令
***/
byte ModBus_setRegister(ModBus_parameter* ModBus_para, uint16_t address, uint16_t data, void(*SetReponseHandler)(uint16_t, uint16_t))
{
	MODBUS_FRAME_T* pFrame = addFrame(ModBus_para);
	pFrame->type = WRITE_SINGLE_REGISTER;
	pFrame->responseSize = 0;
	pFrame->responseHandler = SetReponseHandler;
	pFrame->address = address;
	pFrame->count = 1;
	if (ModBus_para->m_modeType == ASCII)
	{
		pFrame->data[pFrame->size++] = ':';
	}
	pFrame->data[pFrame->size++] = ModBus_para->m_address; // 设备地址
	pFrame->data[pFrame->size++] = WRITE_SINGLE_REGISTER; // 功能码, 写入单寄存器
	pFrame->data[pFrame->size++] = (address >> 8) & 0x0FF; // 寄存器首地址高位
	pFrame->data[pFrame->size++] = address & 0x0FF; // 寄存器首地址低位
	pFrame->data[pFrame->size++] = (data >> 8) & 0x0FF; // 数据高位
	pFrame->data[pFrame->size++] = data & 0x0FF; // 数据低位
	switch (ModBus_para->m_modeType)
	{
	case ASCII:
		pFrame->size = GenLRC(pFrame->data + 1, pFrame->size - 1) + 1; // 不包括起始字符
		pFrame->size = bin2char_s(pFrame->data + 1, pFrame->size - 1, MODBUS_BUFFER_SIZE) + 1;
		pFrame->data[pFrame->size++] = '\r'; // 结束字符
		pFrame->data[pFrame->size++] = '\n'; // 结束字符
		pFrame->responseSize = 17; // 返回帧需要的字节数
		break;
	case RTU:
		pFrame->size = GenCRC16(pFrame->data, pFrame->size);
		pFrame->responseSize = 8; // 返回帧需要的字节数
		break;
	default:
		break;
	}

	return pFrame->index;
}

/** 写多个寄存器 **/
/*** 参数 ***
** address: 寄存器首地址
** data: 待写入数据
** count: 待写入寄存器个数
** SetReponseHandler: 写入结果回调函数, 传入参数(uint16_t address, uint16_t count), 参数包括首地址和寄存器个数
** 返回0表示发送成功, 返回1表示忙未发送
***/
byte ModBus_setRegisters(ModBus_parameter* ModBus_para, uint16_t address, uint16_t* data, uint16_t count, void(*SetReponseHandler)(uint16_t, uint16_t))
{
	MODBUS_FRAME_T* pFrame = addFrame(ModBus_para);
	pFrame->type = WRITE_MULTI_REGISTER;
	pFrame->responseSize = 0;
	pFrame->responseHandler = SetReponseHandler;
	pFrame->address = address;
	pFrame->count = count;
	if (ModBus_para->m_modeType == ASCII)
	{
		pFrame->data[pFrame->size++] = ':';
	}
	pFrame->data[pFrame->size++] = ModBus_para->m_address; // 设备地址
	pFrame->data[pFrame->size++] = WRITE_MULTI_REGISTER; // 功能码, 写入多寄存器
	pFrame->data[pFrame->size++] = (address >> 8) & 0x0FF; // 寄存器首地址高位
	pFrame->data[pFrame->size++] = address & 0x0FF; // 寄存器首地址低位
	pFrame->data[pFrame->size++] = (count >> 8) & 0x0FF; // 寄存器个数高位
	pFrame->data[pFrame->size++] = count & 0x0FF; // 寄存器个数低位
	switch (ModBus_para->m_modeType)
	{
	case ASCII:
		pFrame->data[pFrame->size++] = count * 4; // 数据字节数
		break;
	case RTU:
		pFrame->data[pFrame->size++] = count * 2; // 数据字节数
		break;
	default:
		break;
	}
	if (count > ModBus_para->m_registerAcessLimit || pFrame->size + 2 * count + 2 > MODBUS_BUFFER_SIZE) // 如果超出最大数据量, 不发送, 立即调用回调函数
	{
		if (SetReponseHandler)
		{
			ModBus_para->m_sendFramesN--;
			(*(SetReponseHandler))(address, 0);
		}
		return 0;
	}
	for (uint16_t i = 0; i < count; i++)
	{
		pFrame->data[pFrame->size++] = (data[i] >> 8) & 0x0FF; // 数据高位
		pFrame->data[pFrame->size++] = data[i] & 0x0FF; // 数据低位
	}
	switch (ModBus_para->m_modeType)
	{
	case ASCII:
		pFrame->size = GenLRC(pFrame->data + 1, pFrame->size - 1) + 1; // 不包括起始字符
		pFrame->size = bin2char_s(pFrame->data + 1, pFrame->size - 1, MODBUS_BUFFER_SIZE) + 1;
		pFrame->data[pFrame->size++] = '\r'; // 结束字符
		pFrame->data[pFrame->size++] = '\n'; // 结束字符
		pFrame->responseSize = 17; // 返回帧需要的字节数
		break;
	case RTU:
		pFrame->size = GenCRC16(pFrame->data, pFrame->size);
		pFrame->responseSize = 8; // 返回帧需要的字节数
		break;
	default:
		break;
	}

	return pFrame->index;
}


// 接收数据结束, 处理数据, 存在有效数据返回1, 否则返回0
static byte ModBus_parseReveivedBuff(ModBus_parameter* ModBus_para)
{
	size_t restSize;
	MODBUS_FRAME_T* pFrame = NULL;
	if (ModBus_para->m_sendFramesN > 0)
	{
		pFrame = ModBus_para->m_sendFrames;
	}
	else // 如果没有等待返回帧, 则不处理数据
	{
		ModBus_para->m_pBeginReceiveBufferTmp = ModBus_para->m_pEndReceiveBufferTmp;
		ModBus_para->m_receiveFrameBufferLen = 0;
		return 0;
	}

	if (!ModBus_detectFrame(ModBus_para, &restSize))
	{
		return 0;
	}

	MODBUS_DELAY_DEBUG(("Frame Delay %d\n", millis() - pFrame->time));
	// 判断功能码
	switch (ModBus_para->m_receiveFrameBuffer[1])
	{
	case READ_REGISTER:
	{
		u8 count = ModBus_para->m_receiveFrameBuffer[2];
		MODBUS_DEBUG(("ModBus read reg response\n"));
		if (count % 2 != 0 || pFrame->type != READ_REGISTER || count != pFrame->count * 2) // 数据异常
		{
			// 保留的未处理的数据
			memcpy(ModBus_para->m_receiveFrameBuffer, ModBus_para->m_receiveFrameBuffer + ModBus_para->m_receiveFrameBufferLen, restSize);
			ModBus_para->m_receiveFrameBufferLen = restSize;
			return 0;
		}
		count >>= 1; // 除2
		if (count > ModBus_para->m_registerAcessLimit)
		{
			count = ModBus_para->m_registerAcessLimit;
		}
		for (size_t i = 0; i < count; i++)
		{
			ModBus_para->m_registerData[i] = (((uint16_t)ModBus_para->m_receiveFrameBuffer[3 + (i << 1)]) << 8) + ModBus_para->m_receiveFrameBuffer[4 + (i << 1)];
		}
		ModBus_para->m_registerCount = count;

		// 回调函数
		if (pFrame->responseHandler)
		{
			(*(GetReponseHandler_T)(pFrame->responseHandler))(ModBus_para->m_registerData, count);
		}
		break;
	}
	case WRITE_SINGLE_REGISTER:
	{
		uint16_t address = (ModBus_para->m_receiveFrameBuffer[2] << 8) + ModBus_para->m_receiveFrameBuffer[3];
		uint16_t data = (ModBus_para->m_receiveFrameBuffer[4] << 8) + ModBus_para->m_receiveFrameBuffer[5];
		uint16_t dataSent;
		MODBUS_DEBUG(("ModBus write 0x%04x %d response\n", address, data));
		if (ModBus_para->m_modeType == ASCII)
		{
			byte data[4];
			memcpy(data, pFrame->data + 9, 4);
			char2bin(data, 4);
			dataSent = ((uint16_t)data[0] << 8) + data[1];
		}
		else if (ModBus_para->m_modeType == RTU)
		{
			dataSent = (pFrame->data[4] << 8) + pFrame->data[5];
		}
		if (pFrame->type != WRITE_SINGLE_REGISTER || address != pFrame->address || dataSent != data) // 数据异常
		{
			// 保留的未处理的数据
			memcpy(ModBus_para->m_receiveFrameBuffer, ModBus_para->m_receiveFrameBuffer + ModBus_para->m_receiveFrameBufferLen, restSize);
			ModBus_para->m_receiveFrameBufferLen = restSize;
			return 0;
		}

		// 回调函数
		if (pFrame->responseHandler)
		{
			(*(SetReponseHandler_T)(pFrame->responseHandler))(address, 1);
		}
		break;
	}
	case WRITE_MULTI_REGISTER:
	{
		uint16_t address = (ModBus_para->m_receiveFrameBuffer[2] << 8) + ModBus_para->m_receiveFrameBuffer[3];
		uint16_t count = (ModBus_para->m_receiveFrameBuffer[4] << 8) + ModBus_para->m_receiveFrameBuffer[5];
		MODBUS_DEBUG(("ModBus write 0x%04x %d regs response\n", address, count));
		if (pFrame->type != WRITE_MULTI_REGISTER || address != pFrame->address || count != pFrame->count) // 数据异常
		{
			// 保留的未处理的数据
			memcpy(ModBus_para->m_receiveFrameBuffer, ModBus_para->m_receiveFrameBuffer + ModBus_para->m_receiveFrameBufferLen, restSize);
			ModBus_para->m_receiveFrameBufferLen = restSize;
			return 0;
		}

		// 回调函数
		if (pFrame->responseHandler)
		{
			(*(SetReponseHandler_T)(pFrame->responseHandler))(address, count);
		}
		break;
	}
	default:
		memcpy(ModBus_para->m_receiveFrameBuffer, ModBus_para->m_receiveFrameBuffer + ModBus_para->m_receiveFrameBufferLen, restSize);
		ModBus_para->m_receiveFrameBufferLen = restSize;
		return 0;
		break;
	}

	memcpy(ModBus_para->m_receiveFrameBuffer, ModBus_para->m_receiveFrameBuffer + ModBus_para->m_receiveFrameBufferLen, restSize);
	ModBus_para->m_receiveFrameBufferLen = restSize;

	// 移除已返回指令
	memcpy(ModBus_para->m_sendFrames, ModBus_para->m_sendFrames + 1, (--ModBus_para->m_sendFramesN) * sizeof(MODBUS_FRAME_T));
	ModBus_para->m_waitingResponse = 0;

	return 1;
}

static void sendFrame_loop(ModBus_parameter* ModBus_para)
{
	u32 now = millis();
	if (ModBus_para->m_waitingResponse && now - ModBus_para->m_lastSentTime < ModBus_para->m_sendTimeout || ModBus_para->m_sendFramesN == 0) // 等待返回帧未超时, 或没有待发送数据
	{
		return;
	}
	if (ModBus_para->m_waitingResponse && now - ModBus_para->m_lastSentTime >= ModBus_para->m_sendTimeout) // 等待返回帧超时
	{
		MODBUS_FRAME_T* pFrame = ModBus_para->m_sendFrames;
		MODBUS_DELAY_DEBUG(("Frame Timeout %d\n", millis() - pFrame->time));
		if (pFrame->responseHandler) // 调用回调, 传入参数(0,0)
		{
			switch (pFrame->type)
			{
			case READ_REGISTER:
				(*(GetReponseHandler_T)(pFrame->responseHandler))(0, 0);
				break;
			case WRITE_SINGLE_REGISTER:
				(*(SetReponseHandler_T)(pFrame->responseHandler))(0, 0);
				break;
			case WRITE_MULTI_REGISTER:
				(*(SetReponseHandler_T)(pFrame->responseHandler))(0, 0);
				break;
			default:
				break;
			}
		}
			
		memcpy(ModBus_para->m_sendFrames, ModBus_para->m_sendFrames + 1, (--ModBus_para->m_sendFramesN) * sizeof(MODBUS_FRAME_T)); // 移除已发送数据包
		ModBus_para->m_waitingResponse = 0;
	}
	if (!ModBus_para->m_waitingResponse && ModBus_para->m_sendFramesN > 0) // 不在等待返回帧且有待发送数据包, 则发送
	{
		MODBUS_FRAME_T* pFrame = ModBus_para->m_sendFrames;
		if (ModBus_para->m_faston) // 如果是快速模式, 则只执行最新的指令
		{
			ModBus_para->m_sendFrames[0] = ModBus_para->m_sendFrames[ModBus_para->m_sendFramesN - 1];
			ModBus_para->m_sendFramesN = 1;
		}
		if (ModBus_para->m_SendHandler != NULL)
		{
			(*ModBus_para->m_SendHandler)(pFrame->data, pFrame->size);
			ModBus_para->m_waitingResponse = 1;
			ModBus_para->m_lastSentTime = millis();
		}
	}
}

void ModBus_Master_loop(ModBus_parameter* ModBus_para)
{
	u32 now = millis();

	if (ModBus_para->m_pBeginReceiveBufferTmp != ModBus_para->m_pEndReceiveBufferTmp)
	{
		ModBus_parseReveivedBuff(ModBus_para); // 处理接收到的数据
	}
	if (now - ModBus_para->m_lastReceivedTime > ModBus_para->m_receiveTimeout) // 接收超时, 处理数据并重置
	{
		ModBus_parseReveivedBuff(ModBus_para); // 处理接收到的数据
		ModBus_para->m_receiveFrameBufferLen = 0;
		ModBus_para->m_lastReceivedTime = millis();
	}

	sendFrame_loop(ModBus_para);
}
#endif

#ifdef MODBUS_SLAVE

void ModBus_attachRegisterHandler(ModBus_parameter* ModBus_para, size_t(*GetRegisterHandler)(uint16_t, uint16_t, uint16_t*), size_t(*SetRegisterHandler)(uint16_t, uint16_t, uint16_t*))
{
	ModBus_para->m_GetRegisterHandler = GetRegisterHandler;
	ModBus_para->m_SetRegisterHandler = SetRegisterHandler;
}

/** 读取寄存器返回帧 **/
/*** 参数 ***
** address: 寄存器首地址
** count: 读取寄存器个数
** GetReponseHandler: 读取结果回调函数, 传入参数(uint16_t* buff, uint16_t buffLen)
***/
static void ModBus_getRegister_Slave(ModBus_parameter* ModBus_para, uint16_t address, uint8_t count)
{
	ModBus_para->m_sendFrameBufferLen = 0;
	if (ModBus_para->m_modeType == ASCII)
	{
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = ':';
	}
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = ModBus_para->m_address; // 设备地址
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = READ_REGISTER; // 功能码, 读寄存器

	if (count > ModBus_para->m_registerAcessLimit || ModBus_para->m_sendFrameBufferLen + 2 * count + 3 > MODBUS_BUFFER_SIZE) // 如果超出最大数据量
	{
		count = 0;
	}

	count = (uint8_t)(*(ModBus_para->m_GetRegisterHandler))(address, count, ModBus_para->m_registerData);
	ModBus_para->m_registerCount = count;
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = count * 2; // 字节数 = 读寄存器个数 * 2
	for (uint16_t i = 0; i < count; i++)
	{
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = (ModBus_para->m_registerData[i] >> 8) & 0x0FF; // 数据高位
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = ModBus_para->m_registerData[i] & 0x0FF; // 数据低位
	}
	switch (ModBus_para->m_modeType)
	{
	case ASCII:
		ModBus_para->m_sendFrameBufferLen = GenLRC(ModBus_para->m_sendFrameBuffer + 1, ModBus_para->m_sendFrameBufferLen - 1) + 1; // 不包括起始字符
		ModBus_para->m_sendFrameBufferLen = bin2char_s(ModBus_para->m_sendFrameBuffer + 1, ModBus_para->m_sendFrameBufferLen - 1, MODBUS_BUFFER_SIZE) + 1;
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = '\r'; // 结束字符
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = '\n'; // 结束字符
		break;
	case RTU:
		ModBus_para->m_sendFrameBufferLen = GenCRC16(ModBus_para->m_sendFrameBuffer, ModBus_para->m_sendFrameBufferLen);
		break;
	default:
		break;
	}
	if (ModBus_para->m_SendHandler != NULL && ModBus_para->m_sendFrameBufferLen > 0)
	{
		(*ModBus_para->m_SendHandler)(ModBus_para->m_sendFrameBuffer, ModBus_para->m_sendFrameBufferLen);
		ModBus_para->m_lastSentTime = millis();
	}
}

/** 写单个寄存器返回帧 **/
/*** 参数 ***
** address: 寄存器首地址
** data: 待写入数据
** SetReponseHandler: 写入结果回调函数, 传入参数(uint16_t address, uint16_t count), 参数包括首地址和寄存器个数
***/
static void ModBus_setRegister_Slave(ModBus_parameter* ModBus_para, uint16_t address, uint16_t data)
{
	ModBus_para->m_sendFrameBufferLen = 0;
	if (ModBus_para->m_modeType == ASCII)
	{
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = ':';
	}
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = ModBus_para->m_address; // 设备地址
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = WRITE_SINGLE_REGISTER; // 功能码, 读寄存器

	if ((*(ModBus_para->m_SetRegisterHandler))(address, 1, &data) == 0) // 如果写入错误, 数据取反后返回, 以便主机判断
	{
		data = ~data;
	}
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = (address >> 8) & 0x0FF; // 寄存器首地址高位
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = address & 0x0FF; // 寄存器首地址低位
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = (data >> 8) & 0x0FF; // 数据高位
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = data & 0x0FF; // 数据低位

	switch (ModBus_para->m_modeType)
	{
	case ASCII:
		ModBus_para->m_sendFrameBufferLen = GenLRC(ModBus_para->m_sendFrameBuffer + 1, ModBus_para->m_sendFrameBufferLen - 1) + 1; // 不包括起始字符
		ModBus_para->m_sendFrameBufferLen = bin2char_s(ModBus_para->m_sendFrameBuffer + 1, ModBus_para->m_sendFrameBufferLen - 1, MODBUS_BUFFER_SIZE) + 1;
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = '\r'; // 结束字符
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = '\n'; // 结束字符
		break;
	case RTU:
		ModBus_para->m_sendFrameBufferLen = GenCRC16(ModBus_para->m_sendFrameBuffer, ModBus_para->m_sendFrameBufferLen);
		break;
	default:
		break;
	}
	if (ModBus_para->m_SendHandler != NULL && ModBus_para->m_sendFrameBufferLen > 0)
	{
		(*ModBus_para->m_SendHandler)(ModBus_para->m_sendFrameBuffer, ModBus_para->m_sendFrameBufferLen);
		ModBus_para->m_lastSentTime = millis();
	}
}

/** 写多个寄存器返回帧 **/
/*** 参数 ***
** address: 寄存器首地址
** data: 待写入数据
** count: 待写入寄存器个数
** SetReponseHandler: 写入结果回调函数, 传入参数(uint16_t address, uint16_t count), 参数包括首地址和寄存器个数
***/
static void ModBus_setRegisters_Slave(ModBus_parameter* ModBus_para, uint16_t address, uint16_t* data, uint16_t count)
{
	ModBus_para->m_sendFrameBufferLen = 0;
	if (ModBus_para->m_modeType == ASCII)
	{
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = ':';
	}
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = ModBus_para->m_address; // 设备地址
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = WRITE_MULTI_REGISTER; // 功能码, 读寄存器

	count = (uint16_t)(*(ModBus_para->m_SetRegisterHandler))(address, count, data);
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = (address >> 8) & 0x0FF; // 首地址高位
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = address & 0x0FF; // 首地址低位
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = (count >> 8) & 0x0FF; // 个数高位
	ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = count & 0x0FF; // 个数低位

	switch (ModBus_para->m_modeType)
	{
	case ASCII:
		ModBus_para->m_sendFrameBufferLen = GenLRC(ModBus_para->m_sendFrameBuffer + 1, ModBus_para->m_sendFrameBufferLen - 1) + 1; // 不包括起始字符
		ModBus_para->m_sendFrameBufferLen = bin2char_s(ModBus_para->m_sendFrameBuffer + 1, ModBus_para->m_sendFrameBufferLen - 1, MODBUS_BUFFER_SIZE) + 1;
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = '\r'; // 结束字符
		ModBus_para->m_sendFrameBuffer[ModBus_para->m_sendFrameBufferLen++] = '\n'; // 结束字符
		break;
	case RTU:
		ModBus_para->m_sendFrameBufferLen = GenCRC16(ModBus_para->m_sendFrameBuffer, ModBus_para->m_sendFrameBufferLen);
		break;
	default:
		break;
	}
	if (ModBus_para->m_SendHandler != NULL && ModBus_para->m_sendFrameBufferLen > 0)
	{
		(*ModBus_para->m_SendHandler)(ModBus_para->m_sendFrameBuffer, ModBus_para->m_sendFrameBufferLen);
		ModBus_para->m_lastSentTime = millis();
	}
}

// 接收数据结束, 处理数据, 存在有效数据返回1, 否则返回0
static byte ModBus_parseReveivedBuff_Slave(ModBus_parameter* ModBus_para)
{
	size_t restSize;
	if (!ModBus_detectFrame(ModBus_para, &restSize))
	{
		return 0;
	}

	// 判断功能码
	switch (ModBus_para->m_receiveFrameBuffer[1])
	{
	case READ_REGISTER:
	{
		uint16_t address = (ModBus_para->m_receiveFrameBuffer[2] << 8) + ModBus_para->m_receiveFrameBuffer[3];
		uint16_t count = (ModBus_para->m_receiveFrameBuffer[4] << 8) + ModBus_para->m_receiveFrameBuffer[5];
		if (count > ModBus_para->m_registerAcessLimit)
		{
			count = 0;
		}
		ModBus_getRegister_Slave(ModBus_para, address, (uint8_t)count);
		break;
	}
	case WRITE_SINGLE_REGISTER:
	{
		uint16_t address = (ModBus_para->m_receiveFrameBuffer[2] << 8) + ModBus_para->m_receiveFrameBuffer[3];
		uint16_t data = (ModBus_para->m_receiveFrameBuffer[4] << 8) + ModBus_para->m_receiveFrameBuffer[5];
		ModBus_setRegister_Slave(ModBus_para, address, data);
		break;
	}
	case WRITE_MULTI_REGISTER:
	{
		uint16_t address = (ModBus_para->m_receiveFrameBuffer[2] << 8) + ModBus_para->m_receiveFrameBuffer[3];
		uint16_t count = (ModBus_para->m_receiveFrameBuffer[4] << 8) + ModBus_para->m_receiveFrameBuffer[5];
		//uint8_t size = ModBus_para->m_receiveFrameBuffer[6];
		if (count > ModBus_para->m_registerAcessLimit)
		{
			count = 0;
		}
		for (uint16_t i = 0; i < count; i++)
		{
			ModBus_para->m_registerData[i] = ((uint16_t)(*(ModBus_para->m_receiveFrameBuffer + 7 + i * 2)) << 8) + (uint16_t)(*(ModBus_para->m_receiveFrameBuffer + 8 + i * 2));
		}
		ModBus_para->m_registerCount = count;
		ModBus_setRegisters_Slave(ModBus_para, address, ModBus_para->m_registerData, count);
		break;
	}
	default:
		assert(0);
		memcpy(ModBus_para->m_receiveFrameBuffer, ModBus_para->m_receiveFrameBuffer + ModBus_para->m_receiveFrameBufferLen, restSize);
		ModBus_para->m_receiveFrameBufferLen = restSize;
		return 0;
		break;
	}
	memcpy(ModBus_para->m_receiveFrameBuffer, ModBus_para->m_receiveFrameBuffer + ModBus_para->m_receiveFrameBufferLen, restSize);
	ModBus_para->m_receiveFrameBufferLen = restSize;
	return 1;
}

void ModBus_Slave_loop(ModBus_parameter* ModBus_para)
{
	u32 now = millis();

	if (ModBus_para->m_pBeginReceiveBufferTmp != ModBus_para->m_pEndReceiveBufferTmp)
	{
		ModBus_parseReveivedBuff_Slave(ModBus_para); // 处理接收到的数据
	}
	if (now - ModBus_para->m_lastReceivedTime > ModBus_para->m_receiveTimeout) // 接收超时, 处理数据并重置
	{
		ModBus_parseReveivedBuff_Slave(ModBus_para); // 处理接收到的数据
		ModBus_para->m_receiveFrameBufferLen = 0;
	}
}
#endif

#ifdef _UNIT_TEST
#include <string.h>
#include <stdio.h>
#include <windows.h>
ModBus_parameter modBus_master_test, modBus_slave_test;
int t = 0;
static int millis()
{
	return t;
}

static void OutputData_master(byte* data, size_t len)
{
	int t = millis();
	switch (modBus_master_test.m_modeType)
	{
	case ASCII:
	{
		char strtmp[1000];
		assert(len < 1000);
		strncpy(strtmp, data, len);
		strtmp[len] = 0;
		printf("master send: %s\n", strtmp);
		break;
	}
	case RTU:
	{
		char strtmp[1000];
		for (size_t i = 0; i < len; i++)
			sprintf(strtmp + i * 2, "%02x", data[i]);
		printf("master send: %s\n", strtmp);

		break;
	}
	default:
		assert(0);
		break;
	}

	for (size_t i = 0; i < len; i++)
	{
		ModBus_readByteFromOuter(&modBus_slave_test, data[i]);
	}
}

static void OutputData_slave(byte* data, size_t len)
{
	switch (modBus_slave_test.m_modeType)
	{
	case ASCII:
	{
		char strtmp[1000];
		assert(len < 1000);
		strncpy(strtmp, data, len);
		strtmp[len] = 0;
		printf("slave send: %s\n", strtmp);
		break;
	}
	case RTU:
	{
		char strtmp[1000];
		for (size_t i = 0; i < len; i++)
			sprintf(strtmp + i * 2, "%02x", data[i]);
		printf("slave send: %s\n", strtmp);
		break;
	}
	default:
		assert(0);
		break;
	}

	for (int i = 0; i < len; i++)
	{
		ModBus_readByteFromOuter(&modBus_master_test, data[i]);
	}
}

uint16_t g_registerData[50];
uint16_t g_address = 0, g_count = 0;

static size_t getReg(uint16_t address, uint16_t n, uint16_t* data)
{
	for (uint16_t i = 0; i < n; i++)
	{
		data[i] = g_registerData[address + i];
	}
	return n;
}

static size_t setReg(uint16_t address, uint16_t n, uint16_t* data)
{
	for (uint16_t i = 0; i < n; i++)
	{
		g_registerData[address + i] = data[i];
	}
	return n;
}

void master_printReg(uint16_t* data, uint16_t count)
{
	char strtmp[1000];
	assert(count == g_count);
	for (uint16_t i = 0; i < count; i++)
		sprintf(strtmp + i * 4, "%04x", data[i]);
	printf("register data: %s\n", strtmp);
}

void master_printSetReg(uint16_t address, uint16_t count)
{
	assert(address == g_address);
	assert(count == g_count);
	printf("set register: address %d, count %d\n", address, count);
}

void unit_test()
{
	// 主机配置
	ModBus_Setting_T modbusSetting;
	modbusSetting.address = 0x01;
	modbusSetting.baudRate = 9600;
	modbusSetting.frameType = ASCII;
	modbusSetting.register_access_limit = 5;
	modbusSetting.sendHandler = OutputData_master;
	ModBus_setup(&modBus_master_test, modbusSetting);
	ModBus_setTimeout(&modBus_master_test, 5, 5);

	// 从机配置
	modbusSetting.address = 0x01;
	modbusSetting.baudRate = 9600;
	modbusSetting.frameType = ASCII;
	modbusSetting.register_access_limit = 5;
	modbusSetting.sendHandler = OutputData_slave;
	ModBus_setup(&modBus_slave_test, modbusSetting);
	ModBus_setTimeout(&modBus_slave_test, 5, 5);
	ModBus_attachRegisterHandler(&modBus_slave_test, getReg, setReg);

	// 初始化虚拟寄存器
	for (int i = 0; i < 10; i++)
	{
		g_registerData[i] = -i;
	}

	for (int i = 0; i < 1000; i++)
	{
		// 测试读寄存器
		g_address = 0;
		g_count = 5;
		ModBus_getRegister(&modBus_master_test, g_address, g_count, master_printReg);
		ModBus_Master_loop(&modBus_master_test);
		t += 10;
		ModBus_Slave_loop(&modBus_slave_test);
		ModBus_Master_loop(&modBus_master_test);

		// 测试写单寄存器
		g_address = 2;
		g_count = 1;
		ModBus_setRegister(&modBus_master_test, g_address, 0x0005, master_printSetReg);
		ModBus_Master_loop(&modBus_master_test);
		t += 10;
		ModBus_Slave_loop(&modBus_slave_test);
		ModBus_Master_loop(&modBus_master_test);

		// 测试读寄存器
		g_address = 1;
		g_count = 3;
		ModBus_getRegister(&modBus_master_test, g_address, g_count, master_printReg);
		ModBus_Master_loop(&modBus_master_test);
		t += 10;
		ModBus_Slave_loop(&modBus_slave_test);
		ModBus_Master_loop(&modBus_master_test);

		// 测试写多寄存器
		{
			uint16_t data[] = { 1,2,3,4 };
			g_address = 0;
			g_count = 4;
			ModBus_setRegisters(&modBus_master_test, g_address, data, g_count, master_printSetReg);
		}
		ModBus_Master_loop(&modBus_master_test);
		t += 10;
		ModBus_Slave_loop(&modBus_slave_test);
		ModBus_Master_loop(&modBus_master_test);

		// 测试读寄存器
		g_address = 0;
		g_count = 5;
		ModBus_getRegister(&modBus_master_test, g_address, g_count, master_printReg);
		ModBus_Master_loop(&modBus_master_test);
		t += 10;
		ModBus_Slave_loop(&modBus_slave_test);
		ModBus_Master_loop(&modBus_master_test);
	}

}

#endif // _UNIT_TEST
