#include "bsp_usart1.h"
#include "bsp_debug_usart.h"
#include "canopen_od.h"
#include "mc_type.h"
SCICOM RsCOM;
RX_EMAIL USART3_EMAIL;           //USART堆栈结构体

/* CRC高位字节值表*/
const unsigned char auchCRCHi[] =
{
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;
/* CRC低位字节值表*/
const unsigned char auchCRCLo[] =
{
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
} ;
/*------------------------------------------------
* @function :创建CRC
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/3
------------------------------------------------*/
unsigned short crc16(unsigned char *puchMsg, unsigned short usDataLen)
{
    unsigned char uchCRCHi = 0xFF;          /* 高CRC字节初始化 */
    unsigned char uchCRCLo = 0xFF;          /* 低CRC 字节初始化 */
    unsigned short uIndex;                  /* CRC循环中的索引 */

    while (usDataLen--)                     /* 传输消息缓冲区 */
    {
        uIndex = uchCRCHi^ *puchMsg++;      /* 计算CRC */
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];
    }
    return (uchCRCHi << 8 | uchCRCLo);
}
/*------------------------------------------------
* @function :CRC检查
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/3
------------------------------------------------*/
uint8_t CRC16_chk(unsigned char *puchMsg, unsigned short usDataLen)
{
    uint16_t CRC16_temp = 0;
    CRC16_temp = crc16(puchMsg, usDataLen);
    if(((uint8_t)(CRC16_temp >> 8) == *(puchMsg + usDataLen)) && ((uint8_t)(CRC16_temp) == *(puchMsg + usDataLen + 1)))
        return TRUE;
    else return FALSE;
}
/*------------------------------------------------
* @function :将创建的CRC校验码更新到数据末尾
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/3
------------------------------------------------*/
uint8_t CRC16_creat(unsigned char *puchMsg, unsigned short usDataLen)
{
    uint16_t CRC16_temp = 0;
    CRC16_temp = crc16(puchMsg, usDataLen);
    *(puchMsg + usDataLen) = (uint8_t)(CRC16_temp >> 8);
    *(puchMsg + usDataLen + 1) = (uint8_t)CRC16_temp;
    return TRUE;
}
/*------------------------------------------------
* @function :准备开始下一次接收
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/3
------------------------------------------------*/
void Maxon_Communicate_clr(void)
{
    /*清除帧头*/
    RsCOM.Sci_RxBuffer[0] = 0;
    /*接收中断状态机初始化*/
    RsCOM.rx_state = WAIT_DLE;
    /*接收数据个数*/
    RsCOM.receivecount = 0;
    /*接收CRC个数*/
    RsCOM.crc_len = 0;
    /*清除接收完成中断*/
    RsCOM.Sci_Monitor_En = SCIMONITOR_DIS;
}
/*------------------------------------------------
* @function :串口响应数据
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/3
------------------------------------------------*/
u8 Maxon_Sci_COM_Send(unsigned char *buffer,u16 len)
{
    u8  i = 0;
    u8  j = 0;
    for(i=0; i<len; i++)
    {
        /* Bytye Stuffing */
        if(i > 2)
        {
            if(*(buffer+i) == Maxon_DLE)
            {
                for(j=0; j<2; j++)
                {
                    USART3->DR = *(buffer+i);
                    while((USART3->SR&0X40) == 0);
                }
            }
            else
            {
                USART3->DR = *(buffer+i);
                while((USART3->SR&0X40) == 0);
            }
        }
        else
        {
            USART3->DR = *(buffer+i);
            while((USART3->SR&0X40) == 0);
        }
    }

    return 0;
}
/*------------------------------------------------
* @function :该函数放在500us当中运行用于监控串口传输数据防止传输出错
* @input    :
* @output   :
* @explain  :50ms未完成丢弃数据重新接收数据
* @author   :wangchuan
* @date     :2023/1/3
------------------------------------------------*/
extern CanTxMsgTypeDef TxMessage;
void Sci_Monitor(void)
{
    if(RsCOM.Sci_Monitor_En == SCIMONITOR_EN)
    {
        /*接收完成标志置位*/
        if(RsCOM.Monitor_state == COMPLETE_RX)
        {
            /*接收完成清除错误计数*/
            RsCOM.Sci_Timing = 0;
            /*串口中断当中等待帧头0x5A,接收到帧头就开始计时，规定时间没有收完数据就报错*/
            RsCOM.Sci_Monitor_En = SCIMONITOR_DIS;
            /*如果队列（先进先出）满*/
            if(USART3_EMAIL.Stack_status == RXBUFFER_STACK_DEPTH)
            {
                /*丢弃当前帧进行下一次接收*/
                Maxon_Communicate_clr();
                return;
            }
            /*CRC检查*/
            if(CRC16_chk(RsCOM.Sci_RxBuffer,9))
            {
                for(u8 i=0; i<RXBUFFER_STACK_DEPTH; i++)
                {
                    /*找到一个空堆栈*/
                    if(USART3_EMAIL.Stack_Pointer[i][STACK_CONDITION] == STACK_NULL)
                    {
                        /*将该队列的状态修改为有数据*/
                        USART3_EMAIL.Stack_Pointer[i][STACK_CONDITION] = STACK_FULL;
                        /*填充优先级第一个进入队列为3（优先级最高）*/
                        USART3_EMAIL.Stack_Pointer[i][STACK_PRIORITY] = RXBUFFER_STACK_DEPTH - USART3_EMAIL.Stack_status;
                        /*帧大小默认为11(1+8+2)个字节固定*/
                        USART3_EMAIL.Stack_Pointer[i][STACK_LENTH] = USART3_EMAIL.frameSize;
                        /*接收到的数据放入队列（先进先出）当中，其中Stack_Pointer为3行4列的二位数组 【行】Depth:3表示最大缓存数据为3条 【列】 1：空/满 2：优先级 3：队列地址 4：队列长度*/
                        Pop_Down_stack(USART3_EMAIL.frameSize,RsCOM.Sci_RxBuffer,*(USART3_EMAIL.Rx_Stack_Buffer+USART3_EMAIL.Stack_Pointer[i][STACK_ADDR]));
                        /*队列状态更新，下一个进来就为他的优先级就减小一个*/
                        USART3_EMAIL.Stack_status++;
                        /*入口参数为TYPE_PRE_BUFFER意思为清除USART3_EMAIL.frameSize，当第一个参数为TYPE_PRE_BUFFER时，第二参数没用*/
                        USART3_STACK_CLR(TYPE_PRE_BUFFER,0);
                        /*复盘是发现问题：串口接收到的数据在放入堆栈之后就可以重新开启接收了，否则缓冲区有什么意义呢？这种在发送给协议栈再进行数据接收有点多余，此处复盘修改之后未测试，使用时需要测试*/
                        Maxon_Communicate_clr();
                        break;
                    }
                }
            }
            /*CRC校验错误回复全F*/
            else
            {
                Usart3_Error_Response(USART_CRC_CHECK_ERROR);
                return;
            }
        }
        /*开始接收就计时，如果超时没有接收完成COMPLETE_RX就报错,回复全0xee*/
        else
        {
            RsCOM.Sci_Timing++;
            /*50ms*/
            if(RsCOM.Sci_Timing > 100)
            {
                Usart3_Error_Response(USART_RECEIVE_TIMEOUT);
                return;
            }
        }
    }
}

/*------------------------------------------------
Function:串口响应错误
Input   :No
Output  :No
Explain :超时和CRC错误
------------------------------------------------*/
void Usart3_Error_Response(u8 errortype)
{
    for(u8 i = 0; i < 9; i++)
    {
        RsCOM.Sci_SxBuffer[i] = errortype;
    }
    CRC16_creat(RsCOM.Sci_SxBuffer,9);
#ifdef UART_DEBUG
    
#else
    Maxon_Sci_COM_Send(RsCOM.Sci_SxBuffer,11);
#endif
    Maxon_Communicate_clr();
}
/*------------------------------------------------
* @function :将串口接收到的数据发送给CAN协议栈进行处理
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/3
------------------------------------------------*/
void Usart3_Data_T0_Canopen(u16 stdid,u16 extid,u16 ide,u16 rtr,u16 dlc )
{
    /*将串口接收到的数据复制进CAN接收Buffer*/
    CANopen_Drive.CanRx_Buffer->StdId = stdid;
    CANopen_Drive.CanRx_Buffer->ExtId = extid;
    CANopen_Drive.CanRx_Buffer->IDE   = ide;
    CANopen_Drive.CanRx_Buffer->RTR   = rtr;
    CANopen_Drive.CanRx_Buffer->DLC   = dlc;
    for(u8 i = 0; i<8; i++)
    {
        CANopen_Drive.CanRx_Buffer->Data[i] = RsCOM.Sci_RxBufferDecode[i+1];
    }
    /*此处为串口和CANopen的接口*/
    MessageType_Check(&CANopen_Drive,CANopen_Drive.CanRx_Buffer->StdId,CANopen_Drive.CanRx_Buffer->RTR);
    /*处理数据完成之后将处理状态改为空闲*/
    RsCOM.Sci_HandleStatus = MACHINE_FREE;
    /*复盘是发现问题：串口接收到的数据在放入堆栈之后就可以重新开启接收了，否则缓冲区有什么意义呢？这种在发送给协议栈再进行数据接收有点多余，此处复盘修改之后未测试，使用时需要测试*/
//    Maxon_Communicate_clr();
}
/*------------------------------------------------
* @function :将队列当中的数据放入数据处理buffer，并发送给CAN协议栈处理
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/3
------------------------------------------------*/
void Sci_Stack_Monitor(unsigned int *process)
{
    unsigned int temp = 0;
    switch(*process)
    {
    case STACK_START:
    {
        //wipe stack data
        memset(&USART3_EMAIL,0,sizeof(USART3_EMAIL));
        //Fill addr to stack_pointer
        for(u8 i=0; i<RXBUFFER_STACK_DEPTH; i++)
        {
            USART3_EMAIL.Stack_Pointer[i][STACK_ADDR] = i;
        }
        *process = STACK_CHECK;
    }
    break;
    case STACK_CHECK:
    {
        /*数据还没处理完成*/
        if(RsCOM.Sci_HandleStatus != MACHINE_FREE)
        {
            break;
        }
        /*如果队列当中有数据*/
        if(USART3_EMAIL.Stack_status != STACK_NULL)
        {
            temp = USART3_EMAIL.Stack_Pointer[0][STACK_PRIORITY];//Choose a demo
            USART3_EMAIL.CurrStack_addr = USART3_EMAIL.Stack_Pointer[0][STACK_ADDR];
            //Find highest priority stack
            for(u8 i=0; i<RXBUFFER_STACK_DEPTH; i++)
            {
                if(USART3_EMAIL.Stack_Pointer[i][STACK_PRIORITY] > temp)
                {
                    temp = USART3_EMAIL.Stack_Pointer[i][STACK_PRIORITY];
                    USART3_EMAIL.CurrStack_addr = USART3_EMAIL.Stack_Pointer[i][STACK_ADDR];
                }
            }
            /*将队列当中的数据赋值给RsCOM.Sci_RxBufferDecode*/
            Pop_Down_stack(USART3_EMAIL.Stack_Pointer[USART3_EMAIL.CurrStack_addr][STACK_LENTH],*(USART3_EMAIL.Rx_Stack_Buffer+USART3_EMAIL.CurrStack_addr),RsCOM.Sci_RxBufferDecode);
//          Sci3Read_count = USART3_EMAIL.Stack_Pointer[USART3_EMAIL.CurrStack_addr][STACK_LENTH];
            USART3_STACK_CLR(TYPE_STACK,USART3_EMAIL.CurrStack_addr);
            RsCOM.Sci_HandleStatus = MACHINE_BUSY;
            *process = STACK_HANDLE;
        }
        else
        {
            break;
        }
    }
    break;
    case STACK_HANDLE:
    {
        /*检查是否为心跳报文*/
        if(RsCOM.Sci_RxBufferDecode[1] == 0x05)
        {
            /*串口中的心跳报文ID不能修改默认为0x77F*/
            Usart3_Data_T0_Canopen(0x077F,0x000,0x0000,0x0000,0x0008);
            break;
        }
        /*CRC检查正确将数据发送给CAN协议栈进行处理*/
        Usart3_Data_T0_Canopen(0x0605,0x000,0x0000,0x0000,0x0008);
        *process = STACK_START;
				
    }
    break;

    default:
        break;
    }
}
/*------------------------------------------------
* @function :把接收到的数据先放入队列（先进先出）
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/3
------------------------------------------------*/
void Pop_Down_stack(unsigned int len,unsigned char *buff_server,unsigned char *buff_client)
{
    unsigned char i = 0;

    for(i=0; i<len; i++)
    {
        *(buff_client+i) = *(buff_server+i);
    }
}
/*------------------------------------------------
* @function :队列清除
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2023/1/3
------------------------------------------------*/
void USART3_STACK_CLR(char treeType,char stack_addr)
{
    switch(treeType)
    {
    case TYPE_PRE_BUFFER:
    {
        RsCOM.Sci_Monitor_En = SCIMONITOR_DIS;
        RsCOM.Sci_Timing = 0;
        USART3_EMAIL.checkCnt = 0;
        USART3_EMAIL.rxcnt = 0;
        USART3_EMAIL.frameSize = 0;
        memset(USART3_EMAIL.RxMail_Prep_buf,0,sizeof(USART3_EMAIL.RxMail_Prep_buf));
    }
    break;
    case TYPE_STACK:
    {
        /*清零当前队列值*/
        memset(*(USART3_EMAIL.Rx_Stack_Buffer + stack_addr),0,sizeof(*(USART3_EMAIL.Rx_Stack_Buffer + stack_addr)));
        USART3_EMAIL.Stack_Pointer[stack_addr][STACK_CONDITION] = STACK_NULL;
        USART3_EMAIL.Stack_Pointer[stack_addr][STACK_PRIORITY] = 0;
        USART3_EMAIL.Stack_Pointer[stack_addr][STACK_LENTH] = 0;
        USART3_EMAIL.CurrStack_addr = 0;
        USART3_EMAIL.Stack_status--;
    }
    break;
    default:
        break;
    }
}
