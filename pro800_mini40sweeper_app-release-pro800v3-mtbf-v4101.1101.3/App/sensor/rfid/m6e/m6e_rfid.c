//标准头文件
#include <string.h>

//stm32头文件
//freertos头文件

//APP头文件
#include "rfid.h"
#include "debug.h"
#include "buf1.h"
#include "buf2.h"
#include "normal_io.h"
#include "iwdg_task.h"
/*
*********************************************************************************************************
*
*	模块名称 : 获取RFID信息
*	文件名称 : m6e_rfid.c
*	版    本 : 	V1.0.0
*	说    明 : 
*			1，负责设置并读取RFID数据，设备的默认波特率是115200，不需要修改，数据高字节在前低字节在后
*			2，本程序启动顺序：与关方相比频段不同，本程序有可能会设置读功率，没有7.2，其他相同
				1，读正在运行的程序
				2，如果返回信息中的 program & 0x01 == 0x01，则需要执行启动应用程序命令
				3，设置频段 = 13，这个值与关方推荐的不同
				4，设置协议 = 5
				5，设置天线 = 0x0101
				6, 读取当前读功率，并判断是否需要重新配置读功率
				7，如果需要重新设置，则重新设置读功率，设置完之后再回到第6步
				8，关闭过滤器
				9，开始读卡（连续盘点），此时会连续往外输出信息
				10，如果需要的话就停止读卡，然后便不再输出信息，此时只需要再次执行开始读卡即可再次打开连续读卡

			3，下面是官方文档给的启动顺序，
				快速与模块建立通讯
				7.1 上电初始化
					a. 获取当前运行程序：FF 00 0C 1D 03
					b. 设置频段：FF 01 97 06 4B BB
					c. 设置协议：FF 02 93 00 05 51 7D
					d. 设置天线：FF 03 91 02 01 01 42 C5
				7.2 单次读卡
					a. 清楚缓存：FF 00 2A 1D 25
					b. 检索标签：FF 05 22 00 00 13 01 F4 2B 19
					C. 从模块缓存获取标签数据：FF 03 29 01 FF 00 1B 03
				7.3 连续盘点
					a. 关闭过滤器：FF 03 9A 01 0C 00 A3 5D
					b. 开始读卡：FF 10 2F 00 00 01 22 00 00 05 07 22 10 00 1B 03 E8 01 FF DD 2B
					C. 停止读卡：FF 03 2F 00 00 02 5E 86
*			
*	修改记录 :
*		版本号  		日期        作者     说明
*		V1.0.0    2018-11-05  杨臣   	正式发布
*		V1.1.0    2018-12-14  杨臣   	1，添加了断连重启功能；2，添加了功率配置接口；3，添加了最大功率检查功能。4，完善了接收判断部分。
*	外设资源占用：
*		串口：串口4的R/T
*		IO口：PA9,PA10
*		DMA：	DMA1_Stream2,DMA_Channel_4
*		NVIC：USART4_IRQn，优先级（4，0）
*********************************************************************************************************
*/


/************************************************************************************************************************************
																宏定义
*************************************************************************************************************************************/


/* Definition for M6E_RFID_UART resources ******************************************/


/* Definition for DMAx resources ********************************************/

#define TIMEOUT_MAX              10000 /* 定义DMA初始化最大的溢出数值 */
#define RFID_DATA_MAX_SIZE 0xFF
#define M6E_RFID_DMA_RX_BUF_SIZE 300 //DMA缓存区大小


/************************************************************************************************************************************
																变量类型定义
*************************************************************************************************************************************/
typedef enum{
	RFID_DO_NOTHING,//不做任何事情
	RFID_WAITING_RET,//操作完成，等待返回信息，每一个正常指令都会有一个返回信息
	RFID_LISTEM,//不操作指令，处于监听状态
	RFID_NEED_OPERATION,//表示当前需要执行操作
}rfidOperationState_t;//rfid的操作状态
typedef enum{
	RFID_CMD_READ_RUNING_PROGRAM,//读正在运行的程序
	RFID_CMD_STARTUP_APP,//启动应用程序
	RFID_CMD_SET_FREQ,//设置频段
	RFID_CMD_SET_PROC,//设置协议
	RFID_CMD_SET_ANTENNA,//设置天线
	RFID_CMD_SET_POWER_OF_READ,//设置读功率，这个是由上位机设置的，如果上位机没有设置则使用默认的值，即不设置
	RFID_CMD_CLOSE_FILTER,//关闭过滤器
	RFID_CMD_CONTINUOUS_REVIEW,//连续盘点
	RFID_CMD_STOP_READ_CARD,//停止读卡
	RFID_CMD_GET_READ_POWER,//获取读功率
}rfidOperationCmd_t;//rfid的操作命令
typedef enum{
	RFID_CODE_READ_RUNING_PROGRAM = 0x0C,//读正在运行的程序
	RFID_CODE_STARTUP_APP = 0X04,//启动应用程序
	RFID_CODE_SET_FREQ = 0X97,//设置频段
	RFID_CODE_SET_PROC = 0X93,//设置协议
	RFID_CODE_SET_ANTENNA = 0X91,//设置天线
	RFID_CODE_SET_POWER_OF_READ = 0X92,//设置读功率，这个是由上位机设置的，如果上位机没有设置则使用默认的值，即不设置
	RFID_CODE_CLOSE_FILTER = 0X9A,//关闭过滤器
	RFID_CODE_2F = 0X2F,//连续盘点或停止读卡
	RFID_CODE_GET_READ_POWER = 0X62,//获取读功率
	RFID_CODE_SINGLE_READ = 0X22,//单次读卡
}rfidOperationCode_t;//rfid的操作码
typedef enum{
	FAULT_NO_TAGS_FOUND = 0x0400,//未找到标签
	FAULT_AFE_NOT_ON = 0x0405,//正确的接收到了指令，但是RFID还未开启
	
}errorMessages_t;
typedef struct{
	rfidOperationState_t operationState;
	rfidOperationCmd_t operationCmd;
	rfidOperationCode_t operationCode;
}rfidOperation_t;

#pragma pack(1)
typedef struct{
	 uint8_t soh;
	 uint8_t len;
	 uint8_t opcode;
	 uint8_t status[2];
	 uint8_t data[RFID_DATA_MAX_SIZE];
	 uint16_t crc;
}dataFromRfid_t;


typedef struct{
	uint8_t ID;
	uint8_t RSSI;//信号强度
}rfidTagInfo_t;


/************************************************************************************************************************************
																变量定义
*************************************************************************************************************************************/
TaskHandle_t taskHandleRfid;//任务句柄


static rfidOperation_t gs_rfidOperation = {RFID_NEED_OPERATION,RFID_CMD_STOP_READ_CARD,RFID_CODE_2F};
static rfidTagInfo_t gs_rfidTagInfo = {0};
static SemaphoreHandle_t gs_rfidDataMutex;
static uint8_t gs_dmaRxBuf[M6E_RFID_DMA_RX_BUF_SIZE];
static dataFromRfid_t gs_dataFromRfid;
//二值信号量，在进入空闲中断后释放
static SemaphoreHandle_t  gs_binarySemaphoreGetNewData = NULL;
//连接状态
//static linkStatus_t gs_linkFlag;//与AVR的连接状态
static ConnectionStatus_t gs_rfidStatus = {5000,2000}; //设置超时时间为2000ms,如果功率过低，而没有读到标签，可能会超过1s才返回数据
//上位机配置信息
static uint16_t gs_cfgReadPower = 0;//上位机配置的RFID读功率
static uint16_t gs_currentReadPower = 0;//当前使用的读功率
static uint16_t gs_maxPower = 0;//最大功率，如果设置功率大于这个值会导致设置不成功，所以如果大于这个值就设置为这个值
//标志是否启动完成
static uint8_t gs_rfidStartUpOk = 0;
//官方提供的crc计算的表
static uint16_t gs_crcTable[] =
{
	0x0000, 0x1021, 0x2042, 0x3063,
	0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b,
	0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
};

/************************************************************************************************************************************
																静态函数声明
*************************************************************************************************************************************/

static uint16_t CalculateCrc(uint8_t *fp_data, uint8_t f_len);
//static void M6E_RFID_Uart_DMA_Rx_Data(void);
static void SendToRfid(uint8_t *fp_data,uint16_t f_len);
//static void CheckRfidLinkState(void);
//static linkStatus_t GetRfidLinkState(void);
//static int InitUartAndDmaRfid(void);
//static int M6E_RFID_UART_DMA_RxConfig(void);
static int AnalyseData(void);
static int RecRfidData(void);
static void SetRfidOperation(rfidOperationState_t f_opSt,rfidOperationCmd_t f_opCmd,rfidOperationCode_t f_opCode);
static void PackSendData(uint8_t opCode,uint8_t *fp_srcData,uint8_t f_srcLen,uint8_t *fp_destData,uint16_t *fp_destLen);


/************************************************************************************************************************************
																硬件初始化
*************************************************************************************************************************************/


/*
*********************************************************************************************************
*	函 数 名: USART_DMA_RxConfig
*	功能说明: 串口的DMA接受配置
*	形    参：BufferDST  目标地址指针
*             BufferSize 缓冲区大小
*	返 回 值: 无
*********************************************************************************************************
*/


/************************************************************************************************************************************
																中断处理
*************************************************************************************************************************************/
//中断顶半部
//void M6E_RFID_UART_IRQHandler(void)
//{
////	if(USART_GetITStatus(M6E_RFID_UART, USART_IT_IDLE) != RESET)  // 空闲中断
////    {
////		M6E_RFID_Uart_DMA_Rx_Data();
////    }
//}

uint32_t cnt21 = 0;	
uint32_t cnt22 = 0;
uint32_t cnt23 = 0;
uint32_t cnt24 = 0;
uint32_t cnt25 = 0;
uint32_t cnt26 = 0;
uint32_t cnt27 = 0;
uint32_t cnt28 = 0;
uint32_t cnt29 = 0;
//static void M6E_RFID_Uart_DMA_Rx_Data(void)
//{
//	BaseType_t higherPriorityTaskWoken = pdFALSE;

////	DMA_Cmd(M6E_RFID_UART_RX_DMA_STREAM, DISABLE);       // 关闭DMA ，防止干扰
//	/* 发送同步信号 */
//	xSemaphoreGiveFromISR(gs_binarySemaphoreGetNewData, &higherPriorityTaskWoken);
//	cnt21++;
////	M6E_RFID_UART->SR;
////	M6E_RFID_UART->DR;	
//	portYIELD_FROM_ISR(higherPriorityTaskWoken);
//}


/************************************************************************************************************************************
							内部静态函数：数据接收、解析、处理、控制等
*************************************************************************************************************************************/


//中断底半部,返回值1=没有获取到数据，0=获取到了数据
static int RecRfidData(void)
{
	int ret = 1;
	uint8_t len = 0;
	uint32_t lenAllData = 0;
		
	
	//1,等待DMA关闭完成
//	while (DMA_GetCmdStatus(M6E_RFID_UART_RX_DMA_STREAM) != DISABLE)
//		DMA_Cmd(M6E_RFID_UART_RX_DMA_STREAM, DISABLE);       // 关闭DMA ，防止干扰
//	//2,清DMA标记
//    DMA_ClearFlag(M6E_RFID_UART_RX_DMA_STREAM, M6E_RFID_UART_RX_DMA_FLAG_ALL);           // 清DMA标志位
//	//3，获取数据长度
//	lenAllData = M6E_RFID_DMA_RX_BUF_SIZE - DMA_GetCurrDataCounter(M6E_RFID_UART_RX_DMA_STREAM); //获得接收到的字节数,总数-剩余待传输
	//4，判断是不是接收到了数据
	cnt22++;
	if(lenAllData != 0)//有时没有收到数据也会进入空闲中断
	{
		cnt23++;
		//初步判断数据是否正常
		len = gs_dmaRxBuf[1];
		if(gs_dmaRxBuf[0] == 0xFF && lenAllData >= (len+7) && len != 0xFF)
		{
			cnt28++;
			gs_dataFromRfid.soh = 0xFF;
			gs_dataFromRfid.len = len;
			gs_dataFromRfid.opcode = gs_dmaRxBuf[2];
			gs_dataFromRfid.status[0] = gs_dmaRxBuf[3];
			gs_dataFromRfid.status[1] = gs_dmaRxBuf[4];
			memcpy(gs_dataFromRfid.data,gs_dmaRxBuf+5,len);
			gs_dataFromRfid.crc = gs_dmaRxBuf[len+6] | (gs_dmaRxBuf[len+5] << 8);
			ret = 0;
		}
		else
			cnt29++;
	}
	else
	{
		cnt26++;
		//清除溢出错误
//		while(USART_GetFlagStatus(M6E_RFID_UART, USART_FLAG_ORE) == SET)
//		{
//			cnt27++;
//			M6E_RFID_UART->DR;
//		}
	}
//	DMA_SetCurrDataCounter(M6E_RFID_UART_RX_DMA_STREAM,M6E_RFID_DMA_RX_BUF_SIZE);    //  重新赋值计数值，必须大于等于最大可能接收到的数据帧数目
//	DMA_Cmd(M6E_RFID_UART_RX_DMA_STREAM, ENABLE);        /* DMA开启，等待数据*/
	return ret;
}

//因为是使用空闲中断来一次性接收至少一包数据，所以，这可以认为数据是完整的。
//0=解析成功，1=失败,-1=错误，2=没有数据
static int AnalyseData(void)
{
	int ret = 1;
	uint16_t crc = 0;
	uint16_t status = (gs_dataFromRfid.status[0] << 8) | gs_dataFromRfid.status[1];
	//1，判断CRC是否正确
	crc = CalculateCrc(&gs_dataFromRfid.len, gs_dataFromRfid.len+4);
	if(crc == gs_dataFromRfid.crc)
	{
		if(gs_rfidOperation.operationState == RFID_WAITING_RET)
		{
			if(status == FAULT_AFE_NOT_ON)//检测到未开启错误，则从第一步读运行程序开始
			{
				ret = 0;
				SetRfidOperation(RFID_NEED_OPERATION,RFID_CMD_READ_RUNING_PROGRAM,RFID_CODE_READ_RUNING_PROGRAM);
				goto out;
			}
			
			switch(gs_rfidOperation.operationCmd)
			{
				case RFID_CMD_READ_RUNING_PROGRAM:
					gs_rfidStartUpOk = 0;
					if((gs_dataFromRfid.data[0] & 0x01) == 0x01)
						SetRfidOperation(RFID_NEED_OPERATION,RFID_CMD_STARTUP_APP,RFID_CODE_STARTUP_APP);
					else
						SetRfidOperation(RFID_NEED_OPERATION,RFID_CMD_SET_FREQ,RFID_CODE_SET_FREQ);
				break;
				case RFID_CMD_STARTUP_APP:
					SetRfidOperation(RFID_NEED_OPERATION,RFID_CMD_READ_RUNING_PROGRAM,RFID_CODE_READ_RUNING_PROGRAM);
				break; 
				case RFID_CMD_SET_FREQ:
					SetRfidOperation(RFID_NEED_OPERATION,RFID_CMD_SET_PROC,RFID_CODE_SET_PROC);
				break; 
				case RFID_CMD_SET_PROC:
					SetRfidOperation(RFID_NEED_OPERATION,RFID_CMD_SET_ANTENNA,RFID_CODE_SET_ANTENNA);
				break;
				case RFID_CMD_SET_ANTENNA:
					SetRfidOperation(RFID_NEED_OPERATION,RFID_CMD_GET_READ_POWER,RFID_CODE_GET_READ_POWER);
				break;
				case RFID_CMD_GET_READ_POWER:
					if(status == 0x00 || status == FAULT_NO_TAGS_FOUND)
					{
						gs_currentReadPower = gs_dataFromRfid.data[2] | (gs_dataFromRfid.data[1] << 8);
						gs_maxPower = gs_dataFromRfid.data[4] | (gs_dataFromRfid.data[3] << 8);
						if(gs_cfgReadPower != 0 && gs_cfgReadPower != gs_currentReadPower)//检查如果上位机配置了功率则需要配置功率，否则直接进行下一步
							SetRfidOperation(RFID_NEED_OPERATION,RFID_CMD_SET_POWER_OF_READ,RFID_CODE_SET_POWER_OF_READ);
						else
							SetRfidOperation(RFID_NEED_OPERATION,RFID_CMD_CLOSE_FILTER,RFID_CODE_CLOSE_FILTER);
					}
					else
					{
						SetRfidOperation(RFID_NEED_OPERATION,RFID_CMD_GET_READ_POWER,RFID_CODE_GET_READ_POWER);
					}
				break;
				case RFID_CMD_SET_POWER_OF_READ:
					SetRfidOperation(RFID_NEED_OPERATION,RFID_CMD_GET_READ_POWER,RFID_CODE_GET_READ_POWER);
				break;
				case RFID_CMD_CLOSE_FILTER:
					SetRfidOperation(RFID_NEED_OPERATION,RFID_CMD_CONTINUOUS_REVIEW,RFID_CODE_2F);
				break;
				case RFID_CMD_CONTINUOUS_REVIEW:
					if(gs_dataFromRfid.opcode == gs_rfidOperation.operationCode || RFID_CODE_SINGLE_READ == gs_dataFromRfid.opcode)
					{
						gs_rfidStartUpOk = 1;
						SetRfidOperation(RFID_LISTEM,RFID_CMD_CONTINUOUS_REVIEW,RFID_CODE_2F);
					}
					else
					{
						gs_rfidOperation.operationState = RFID_NEED_OPERATION;
					}
				break;
				case RFID_CMD_STOP_READ_CARD:
					if(gs_dataFromRfid.opcode != gs_rfidOperation.operationCode)
						gs_rfidOperation.operationState = RFID_NEED_OPERATION;
					else
					{
						if(gs_rfidStartUpOk == 0)
							SetRfidOperation(RFID_NEED_OPERATION,RFID_CMD_READ_RUNING_PROGRAM,RFID_CODE_READ_RUNING_PROGRAM);
						else
							SetRfidOperation(RFID_DO_NOTHING,RFID_CMD_STOP_READ_CARD,RFID_CODE_2F);
					}
				break;
				
				default:
				break;
			}
		}
		else if(gs_rfidOperation.operationState == RFID_LISTEM && RFID_CODE_SINGLE_READ == gs_dataFromRfid.opcode)//监听状态，获取RFID标签信息
		{
			if(status == 0)
			{
				gs_rfidTagInfo.RSSI = gs_dataFromRfid.data[7];
				gs_rfidTagInfo.ID = gs_dataFromRfid.data[37];
			}
			else 
			{
				gs_rfidTagInfo.RSSI = 0;
				gs_rfidTagInfo.ID = 0;
			}
		}
		ret = 0;
	}
	else
		ret = 1;
out:
	return ret;
}


static void SetRfidOperation(rfidOperationState_t f_opSt,rfidOperationCmd_t f_opCmd,rfidOperationCode_t f_opCode)
{
	gs_rfidOperation.operationCmd = f_opCmd;
	gs_rfidOperation.operationCode = f_opCode;
	gs_rfidOperation.operationState = f_opSt;
}


static void RfidOperation(void)//处于等待时开始计时，检查超时情况，处于其他状态时清除计时，超时后需要重发原命令
{
	const uint8_t c_startRead[16] = {0x00,0x00,0x01,0x22,0x00,0x00,0x05,0x07,0x22,0x10,0x00,0x1b,0x03,0xe8,0x01,0xff};
	uint8_t srcData[32] = {0};
	uint8_t srcLen = 0;
	uint8_t destData[32] = {0};
	uint16_t destLen = 0;
	static TickType_t timeOutEndPiont = 0;
	const TickType_t c_maxWaitTime = pdMS_TO_TICKS(100); 

	if(gs_rfidOperation.operationState == RFID_WAITING_RET)//查看是否超时，超时则重新发送
	{
		if(timeOutEndPiont < xTaskGetTickCount())
		{
			gs_rfidOperation.operationState = RFID_NEED_OPERATION;
		}
	}
	if(gs_rfidOperation.operationState == RFID_NEED_OPERATION)//操作并重新设置超时时间
	{
		switch(gs_rfidOperation.operationCmd)
		{
			case RFID_CMD_READ_RUNING_PROGRAM:
				srcLen = 0;
				gs_rfidOperation.operationCode = RFID_CODE_READ_RUNING_PROGRAM;	
				gs_rfidOperation.operationState = RFID_WAITING_RET;
			break;
			case RFID_CMD_STARTUP_APP:
				srcLen = 0;
				gs_rfidOperation.operationCode = RFID_CODE_STARTUP_APP;
				gs_rfidOperation.operationState = RFID_WAITING_RET;
			break; 
			case RFID_CMD_SET_FREQ:
				srcData[0] = 13;
				srcLen = 1;
				gs_rfidOperation.operationCode = RFID_CODE_SET_FREQ;
				gs_rfidOperation.operationState = RFID_WAITING_RET;
			break; 
			case RFID_CMD_SET_PROC:
				srcData[0] = 0x00;
				srcData[1] = 0x05;
				srcLen = 2;
				gs_rfidOperation.operationCode = RFID_CODE_SET_PROC;
				gs_rfidOperation.operationState = RFID_WAITING_RET;
			break;
			case RFID_CMD_SET_ANTENNA:
				srcData[0] = 0x02;
				srcData[1] = 0x01;
				srcData[2] = 0x01;
				srcLen = 3;
				gs_rfidOperation.operationCode = RFID_CODE_SET_ANTENNA;
				gs_rfidOperation.operationState = RFID_WAITING_RET;
			break;
			case RFID_CMD_SET_POWER_OF_READ:
				srcData[0] = gs_cfgReadPower >> 8;
				srcData[1] = gs_cfgReadPower & 0xff;
				srcLen = 2;
				gs_rfidOperation.operationCode = RFID_CODE_SET_POWER_OF_READ;
				gs_rfidOperation.operationState = RFID_WAITING_RET;
			break;
			case RFID_CMD_CLOSE_FILTER:
				srcData[0] = 0x01;
				srcData[1] = 0x0C;
				srcData[2] = 0x00;
				srcLen = 3;
				gs_rfidOperation.operationCode = RFID_CODE_CLOSE_FILTER;
				gs_rfidOperation.operationState = RFID_WAITING_RET;
			break;
			case RFID_CMD_CONTINUOUS_REVIEW:
				srcLen = sizeof(c_startRead);
				memcpy(srcData,c_startRead,srcLen);
				gs_rfidOperation.operationState = RFID_WAITING_RET;
			break;
			case RFID_CMD_STOP_READ_CARD:
				srcData[0] = 0x00;
				srcData[1] = 0x00;
				srcData[2] = 0x02;
				srcLen = 3;
				gs_rfidOperation.operationCode = RFID_CODE_2F;
				gs_rfidOperation.operationState = RFID_WAITING_RET;
			break;
			case RFID_CMD_GET_READ_POWER:
				srcData[0] = 0x01;
				srcLen = 1;
				gs_rfidOperation.operationCode = RFID_CODE_GET_READ_POWER;
				gs_rfidOperation.operationState = RFID_WAITING_RET;
			break;
			default:
			break;
		}
		if(gs_rfidOperation.operationState != RFID_NEED_OPERATION)
		{
			PackSendData(gs_rfidOperation.operationCode,srcData,srcLen,destData,&destLen);
			SendToRfid(destData,destLen);
			timeOutEndPiont = xTaskGetTickCount() + c_maxWaitTime;
		}
	}

}

//f_len= fp_data长度，fp_data数据部分，f_opCode操作码
static void PackSendData(uint8_t opCode,uint8_t *fp_srcData,uint8_t f_srcLen,uint8_t *fp_destData,uint16_t *fp_destLen)
{
	uint16_t crc = 0;
	fp_destData[0] = 0xFF;
	fp_destData[1] = f_srcLen;
	fp_destData[2] = opCode;
	
	if(f_srcLen != 0 & fp_srcData != NULL || fp_destLen != NULL)
		memcpy(fp_destData+3,fp_srcData,f_srcLen);
		
	crc = CalculateCrc(fp_destData+1, f_srcLen + 2);
	
	fp_destData[f_srcLen+3] = (crc >> 8) & 0xff;
	fp_destData[f_srcLen+4] = crc & 0xff;
	
	*fp_destLen = f_srcLen + 5;
}

static void SendToRfid(uint8_t *fp_data,uint16_t f_len)
{
	int i = 0;
	for(i=0;i<f_len;i++)
	{
//		USART_SendData(M6E_RFID_UART, fp_data[i]);
//		while(USART_GetFlagStatus(M6E_RFID_UART, USART_FLAG_TC) == RESET);
	}
}

//这个函数和gs_crcTable是m6e文档中提供的
static uint16_t CalculateCrc(uint8_t *fp_data, uint8_t f_len)
{
	uint16_t crc;
	int i;
	crc = 0xffff;
	for (i = 0; i < f_len ; i++)
	{
		crc = ((crc << 4) | (fp_data[i] >> 4)) ^ gs_crcTable[crc >> 12];
		crc = ((crc << 4) | (fp_data[i] & 0xf)) ^ gs_crcTable[crc >> 12];
	}
	return crc;
}

/************************************************************************************************************************************
											对外提供的数据、状态、配置、控制接口定义
*************************************************************************************************************************************/

uint8_t GetRfidHealthState(void)
{
	
	return GetLinkStatus(&gs_rfidStatus);
}
void GetRfidData(uint8_t *fp_id,uint8_t *fp_rssi)
{
	xSemaphoreTake(gs_rfidDataMutex, portMAX_DELAY);
	*fp_id = gs_rfidTagInfo.ID;
	*fp_rssi = gs_rfidTagInfo.RSSI;
	xSemaphoreGive(gs_rfidDataMutex);
}
void WriteRfidPowerCfgPara(uint32_t f_rfidPower)
{
	if(gs_maxPower < f_rfidPower)
		f_rfidPower = gs_maxPower;
		
	gs_cfgReadPower = f_rfidPower;
}


/************************************************************************************************************************************
												任务+对外的硬件初始化+对外的软件初始化
*************************************************************************************************************************************/

void RfidTask(void *pvParameters)
{
	BaseType_t result;
	int ret = 0;
	const TickType_t c_maxBlockTime = pdMS_TO_TICKS(5); /* 设置最大等待时间为 */
	const TickType_t c_checkResetPeriod = pdMS_TO_TICKS(1000); 
    uint8_t thread_num = (uint8_t)(uint32_t)pvParameters;
    
	TickType_t checkResetTime = 0;
	uint32_t bitPos = (uint32_t)pvParameters;
	while(1)
	{
		/* 喂狗 */
		FeedDog(bitPos);
		//判断连接状态
		CheckRfidLinkState();
		if(!GetLinkStatus(&gs_rfidStatus))
		{
			gs_rfidTagInfo.RSSI = 0;
			gs_rfidTagInfo.ID = 0;
		}
		//1s检查一次，在RFID_LISTEM时，如果断开失联或者当前功率和配置功率不同，则从头开始
		if((gs_rfidOperation.operationState == RFID_LISTEM) &&\
			((!GetLinkStatus(&gs_rfidStatus)) || (gs_cfgReadPower != 0 && gs_cfgReadPower != gs_currentReadPower)))
		{
			if(checkResetTime <= xTaskGetTickCount())
			{
				gs_rfidStartUpOk = 0;
				SetRfidOperation(RFID_NEED_OPERATION,RFID_CMD_STOP_READ_CARD,RFID_CODE_2F);
				checkResetTime = xTaskGetTickCount() + c_checkResetPeriod;
			}
		}
		//读取接收到的数据
		result = xSemaphoreTake(gs_binarySemaphoreGetNewData, (TickType_t)c_maxBlockTime);
		if(result == pdTRUE)
		{
			ret = RecRfidData();
			//有新的数据写入，开始解析
			if(ret == 0)
			{
				if(AnalyseData() == 0)
					ResetConnectedTime(&gs_rfidStatus);
			}
		}
		RfidOperation();
        
        g_thread_call_count[thread_num]++;
	}
}


int InitSoftwareRfid(void)
{
	/* 创建二值信号量，首次创建信号量计数值是0 */
	gs_binarySemaphoreGetNewData = xSemaphoreCreateBinary();
	if(gs_binarySemaphoreGetNewData == NULL)
		return 1;
	//RFID对外数据的互斥量
	gs_rfidDataMutex = xSemaphoreCreateMutex();
	if(gs_rfidDataMutex == NULL)
		return 1;
	return 0;
}





