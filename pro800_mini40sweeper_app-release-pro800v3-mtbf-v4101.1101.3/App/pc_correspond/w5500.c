#include <stdio.h>
#include <string.h>
#include "w5500.h"
#include "socket.h"
#include "spi.h"
#include "apptask.h"
/******************************************************************************************************************************
*             Macro Define Section
*******************************************************************************************************************************/

/***************----- W5500 GPIO定义 -----***************/

#define W5500_RST_PIN			GPIO_Pin_8	//定义W5500的RST引脚
#define W5500_RST_PORT			GPIOA
#define W5500_RST_MODE			GPIO_Mode_OUT
#define W5500_RST_OTYPE			GPIO_OType_PP
#define W5500_RST_SPEED			GPIO_Speed_50MHz
#define W5500_RST_VALID			0


#define W5500_INT_PIN			GPIO_Pin_9	//定义W5500的INT引脚
#define W5500_INT_PORT			GPIOC
#define W5500_INT_MODE			GPIO_Mode_IN
#define W5500_INT_PUPD			GPIO_PuPd_UP
#define W5500_INT_SPEED			GPIO_Speed_50MHz
#define W5500_INT_VALID			0

#if	W5500_RST_VALID
	#define	W5500_RST_ON		GPIO_SetBits(W5500_RST_PORT,W5500_RST_PIN)
	#define	W5500_RST_OFF		GPIO_ResetBits(W5500_RST_PORT,W5500_RST_PIN)
#else
	#define	W5500_RST_ON		GPIO_ResetBits(W5500_RST_PORT,W5500_RST_PIN)
	#define	W5500_RST_OFF		GPIO_SetBits(W5500_RST_PORT,W5500_RST_PIN)
#endif
	
/******************************************************************************************************************************
*             Structure Define Section
*******************************************************************************************************************************/
static CONFIG_MSG  ConfigMsg=
{
   .mac = {0x00,0x08,0xdc,0x11,0x11,0x11},  //mac地址
   .lip = {10,7,5,199},  //IP地址
   .sub = {255,255,255,0},  //子网掩码
   .gw = {10,7,5,1},   //网关
   .dns = {8,8,8,8},       //dns
};

static uint16_t SSIZE[MAX_SOCK_NUM]; /**< Max Tx buffer size by each channel */
static uint16_t RSIZE[MAX_SOCK_NUM]; /**< Max Rx buffer size by each channel */
static uint8_t txsize[MAX_SOCK_NUM] = {2,2,2,2,2,2,2,2};    // 选择8个Socket每个Socket发送缓存的大小，在w5500.c的void sysinit()有设置过程
static uint8_t rxsize[MAX_SOCK_NUM] = {2,2,2,2,2,2,2,2};    // 选择8个Socket每个Socket接收缓存的大小，在w5500.c的void sysinit()有设置过程
/****************************************************************************************************************************
*	Function Name       :
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :
*
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code
*****************************************************************************************************************************/

uint16_t getIINCHIP_RxMAX(uint8_t s)
{
   return RSIZE[s];
}
uint16_t getIINCHIP_TxMAX(uint8_t s)
{
   return SSIZE[s];
}
/****************************************************************************************************************************
*	Function Name       :
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  SPI片选信号设置
*
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code
*****************************************************************************************************************************/
void IINCHIP_CSoff(void)
{
  WIZ_CS(LOW);
}

void IINCHIP_CSon(void)
{
   WIZ_CS(HIGH);
}
/****************************************************************************************************************************
*	Function Name       :  IINCHIP_SpiSendData
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  SPI发送

*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code
*****************************************************************************************************************************/

u8  IINCHIP_SpiSendData(uint8_t dat)
{
   return(SPI2_SendByte(dat));
}
/****************************************************************************************************************************
*	Function Name       :  IINCHIP_WRITE
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  向w5500寄存器写一个字节的数据

*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code
*****************************************************************************************************************************/

void IINCHIP_WRITE( uint32_t addrbsb,  uint8_t data)
{

   IINCHIP_CSoff();                              // CS=0, SPI start
   IINCHIP_SpiSendData( (addrbsb & 0x00FF0000)>>16);// Address byte 1
   IINCHIP_SpiSendData( (addrbsb & 0x0000FF00)>> 8);// Address byte 2
   IINCHIP_SpiSendData( (addrbsb & 0x000000F8) + 4);    // Data write command and Write data length 1
   IINCHIP_SpiSendData(data);                    // Data write (write 1byte data)
   IINCHIP_CSon();                               // CS=1,  SPI end

}

/****************************************************************************************************************************
*	Function Name       :  IINCHIP_READ
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  读取w5500寄存器一个字节的数据
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code
*****************************************************************************************************************************/

uint8_t IINCHIP_READ(uint32_t addrbsb)
{
   uint8_t data = 0;

   IINCHIP_CSoff();                              // CS=0, SPI start
   IINCHIP_SpiSendData( (addrbsb & 0x00FF0000)>>16);// Address byte 1
   IINCHIP_SpiSendData( (addrbsb & 0x0000FF00)>> 8);// Address byte 2
   IINCHIP_SpiSendData( (addrbsb & 0x000000F8))    ;// Data read command and Read data length 1
   data = IINCHIP_SpiSendData(0x00);             // Data read (read 1byte data)
   IINCHIP_CSon();                               // CS=1,  SPI end

   return data;
}
/****************************************************************************************************************************
*	Function Name       :  WriteW5500_Buf
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  将数据写入w5500寄存器， ;
*                        结构为 |地址高字节|地址低字节|控制段|数据段0-n字节
*                        控制段  |7 6 5 4  3| 2 |1 0
*                                |选择Socket|R/W|Mode
*                        Mode    00  可变数据长度  n Byte
*                                01  固定数据长度  1 Byte
*                                10  固定数据长度  2 Byte
*                                11  固定数据长度  4 Byte
* Parameter           :  speed,speed
* Return Code         :  None
* Global Variable     :  None
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code
*****************************************************************************************************************************/

uint16_t WriteW5500_Buf(uint32_t addrbsb,uint8_t* buf,uint16_t len)
{
   uint16_t idx = 0;


   IINCHIP_CSoff();                              // CS=0, SPI start
   IINCHIP_SpiSendData( (addrbsb & 0x00FF0000)>>16);// Address byte 1
   IINCHIP_SpiSendData( (addrbsb & 0x0000FF00)>> 8);// Address byte 2
   IINCHIP_SpiSendData( (addrbsb & 0x000000F8) + 4);    // Data write command and Write data length 1 可变数据长度
   for(idx = 0; idx < len; idx++)                // Write data in loop
   {
     IINCHIP_SpiSendData(buf[idx]);
   }
   IINCHIP_CSon();                               // CS=1, SPI end

   return len;
}
/****************************************************************************************************************************
*	Function Name       :  ReadW5500_Buf
*	Create Date         :  2016/10/15
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  读取W5500寄存器里的值 ;
*
* Parameter           :  speed,speed
* Return Code         :  None
* Global Variable     :  None
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code
*****************************************************************************************************************************/
uint16_t ReadW5500_Buf(uint32_t addrbsb, uint8_t* buf,uint16_t len)
{
  uint16_t idx = 0;

  IINCHIP_CSoff();                                  	// CS=0, SPI开启
  IINCHIP_SpiSendData( (addrbsb & 0x00FF0000)>>16);		// 通过SPI发送16位地址段给MCU
  IINCHIP_SpiSendData( (addrbsb & 0x0000FF00)>> 8);		//
  IINCHIP_SpiSendData( (addrbsb & 0x000000F8));    		// 设置SPI为读操作
  for(idx = 0; idx < len; idx++)                    	// 将buf中的数据通过SPI发送给MCU
  {
    buf[idx] = IINCHIP_SpiSendData(0x00);
  }
  IINCHIP_CSon();                                   	// CS=1, SPI关闭

  return len;																					// 返回已接收数据的长度值
}

/**
@brief  This function is for resetting of the iinchip. Initializes the iinchip to work in whether DIRECT or INDIRECT mode
*/

void iinchip_init(void)
{
  setMR( MR_RST );
#ifdef __DEF_IINCHIP_DBG__
//  printf("MR value is %02x \r\n",IINCHIP_READ_COMMON(MR));
#endif
}
/****************************************************************************************************************************
*	Function Name       :  sysinit
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  设置发送接收缓冲区的大小;
*                        此处设置为2K

*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code
*****************************************************************************************************************************/

/**
@brief  This function set the transmit & receive buffer size as per the channels is used
Note for TMSR and RMSR bits are as follows\n
bit 1-0 : memory size of channel #0 \n
bit 3-2 : memory size of channel #1 \n
bit 5-4 : memory size of channel #2 \n
bit 7-6 : memory size of channel #3 \n
bit 9-8 : memory size of channel #4 \n
bit 11-10 : memory size of channel #5 \n
bit 12-12 : memory size of channel #6 \n
bit 15-14 : memory size of channel #7 \n
Maximum memory size for Tx, Rx in the W5500 is 16K Bytes,\n
In the range of 16KBytes, the memory size could be allocated dynamically by each channel.\n
Be attentive to sum of memory size shouldn't exceed 8Kbytes\n
and to data transmission and receiption from non-allocated channel may cause some problems.\n
If the 16KBytes memory is already  assigned to centain channel, \n
other 3 channels couldn't be used, for there's no available memory.\n
If two 4KBytes memory are assigned to two each channels, \n
other 2 channels couldn't be used, for there's no available memory.\n
*/
void sysinit( uint8_t * tx_size, uint8_t * rx_size  )
{
  int16_t i;
  int16_t ssum,rsum;
  ssum = 0;
  rsum = 0;

  for (i = 0 ; i < MAX_SOCK_NUM; i++)       // Set the size, masking and base address of Tx & Rx memory by each channel
  {
          IINCHIP_WRITE( (Sn_TXMEM_SIZE(i)), tx_size[i]);
          IINCHIP_WRITE( (Sn_RXMEM_SIZE(i)), rx_size[i]);

    SSIZE[i] = (int16_t)(0);
    RSIZE[i] = (int16_t)(0);

// W5500有8个Socket，每个Socket有对应独立的收发缓存区。
// 每个Socket的发送/接收缓存区都在一个16KB的物理发送内存中，初始化分配为2KB。
// 无论给每个Socket分配多大的收/发缓存，都必须在16KB以内。

    if (ssum <= 16384)										// 设置Socket发送缓存空间的大小
    {
       switch( tx_size[i] )
				{
					case 1:
						SSIZE[i] = (int16_t)(1024);			// i=1，tx_size=1KB
					break;
					case 2:
						SSIZE[i] = (int16_t)(2048);			// i=2，tx_size=2KB
					break;
					case 4:
						SSIZE[i] = (int16_t)(4096);			// i=4，tx_size=4KB
					break;
					case 8:
						SSIZE[i] = (int16_t)(8192);			// i=8，tx_size=8KB
					break;
					case 16:
						SSIZE[i] = (int16_t)(16384);		// i=16，tx_size=16KB
					break;
					default :
						RSIZE[i] = (int16_t)(2048);			// 默认i=2，tx_size=2KB
					break;
				}
		}

			if (rsum <= 16384)									// 设置Socket接收缓存空间的大小
			{
					switch( rx_size[i] )
					{
						case 1:
							RSIZE[i] = (int16_t)(1024);		// i=1，rx_size=1KB
						break;
						case 2:
							RSIZE[i] = (int16_t)(2048);		// i=2，rx_size=2KB
						break;
						case 4:
							RSIZE[i] = (int16_t)(4096);		// i=4，rx_size=4KB
						break;
						case 8:
							RSIZE[i] = (int16_t)(8192);		// i=8，rx_size=8KB
						break;
						case 16:
							RSIZE[i] = (int16_t)(16384);	// i=16，rx_size=16KB
						break;
						default :
							RSIZE[i] = (int16_t)(2048);		// 默认i=2，rx_size=2K
						break;
					}
			}
    ssum += SSIZE[i];
    rsum += RSIZE[i];
  }
}

// added

/****************************************************************************************************************************
*	Function Name       :
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  设置网关寄存器
*
* Parameter           :  addr ，a pointer to a 4 -byte array responsible to set the Gateway IP address.
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code
*****************************************************************************************************************************/

void setGAR(uint8_t * addr)
{
    WriteW5500_Buf(GAR0, addr, 4);
}

void getGAR(uint8_t * addr)
{
    ReadW5500_Buf(GAR0, addr, 4);
}

void getGWIP(uint8_t * addr)
{
    ReadW5500_Buf(GAR0, addr, 4);
}

/****************************************************************************************************************************
*	Function Name       :
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  设置子网掩码寄存器
*
* Parameter           :  addr ，a pointer to a 4 -byte array responsible to set the Gateway IP address.
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code
*****************************************************************************************************************************/
void setSUBR(uint8_t * addr)
{
    WriteW5500_Buf(SUBR0, addr, 4);
}

void getSUBR(uint8_t * addr)
{
    ReadW5500_Buf(SUBR0, addr, 4);
}
/****************************************************************************************************************************
*	Function Name       :
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  设置mac地址寄存器
*
* Parameter           :  addr ，a pointer to a 6 -byte array responsible to set the Gateway IP address.
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code
*****************************************************************************************************************************/

void setSHAR(uint8_t * addr )
{
  WriteW5500_Buf(SHAR0, addr, 6);
}

void getSHAR(uint8_t * addr)
{
    ReadW5500_Buf(SHAR0, addr, 6);
}
/****************************************************************************************************************************
*	Function Name       :
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  设置本地IP地址寄存器
*
* Parameter           :  addr ，a pointer to a 4 -byte array responsible to set the Gateway IP address.
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code
*****************************************************************************************************************************/

void setSIPR(uint8_t * addr)
{
    WriteW5500_Buf(SIPR0, addr, 4);
}

void getSIPR(uint8_t * addr)
{
    ReadW5500_Buf(SIPR0, addr, 4);
}
/****************************************************************************************************************************
*	Function Name       :  setkeepalive
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  W5500心跳检测程序，
                         设置Socket在线时间寄存器Sn_KPALVTR，单位为5s
*
* Parameter           :  addr ，a pointer to a 4 -byte array responsible to set the Gateway IP address.
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code
*****************************************************************************************************************************/

void setkeepalive(SOCKET s)
{
  IINCHIP_WRITE(Sn_KPALVTR(s),0x02);  //设置心跳发送周期为10s
}






/****************************************************************************************************************************
*	Function Name       :  setMR
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  设置通用寄存器区模式寄存器，包括ping，复位寄存器等功能

*
* Parameter           :  addr ，a pointer to a 4 -byte array responsible to set the Gateway IP address.
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code
*****************************************************************************************************************************/

void setMR(uint8_t val)
{
  IINCHIP_WRITE(MR,val);
}

/****************************************************************************************************************************
*	Function Name       :  getIR
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  读取通用寄存器区的中断寄存器

*
* Parameter           :  addr ，a pointer to a 4 -byte array responsible to set the Gateway IP address.
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code
*****************************************************************************************************************************/
uint8_t getIR( void )
{
   return IINCHIP_READ(IR);
}
/****************************************************************************************************************************
*	Function Name       :  setRTR
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  设置超时重发发送时间
*                        If there is no response from the peer or delay in response then retransmission
*                        will be there as per RTR (Retry Time-value Register)setting
*
* Parameter           :  addr ，a pointer to a 4 -byte array responsible to set the Gateway IP address.
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code
*****************************************************************************************************************************/
void setRTR(uint16_t timeout)
{
  IINCHIP_WRITE(RTR0,(uint8_t)((timeout & 0xff00) >> 8));
  IINCHIP_WRITE(RTR1,(uint8_t)(timeout & 0x00ff));
}

/****************************************************************************************************************************
*	Function Name       :  setRCR
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  设置超时重发次数
*                       If there is no response from the peer or delay in response then recorded time
*												as per RTR & RCR register seeting then time out will occur.
*
* Parameter           :  addr ，a pointer to a 4 -byte array responsible to set the Gateway IP address.
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code
*****************************************************************************************************************************/
void setRCR(uint8_t retry)
{
  IINCHIP_WRITE(WIZ_RCR,retry);
}

/****************************************************************************************************************************
*	Function Name       :  clearIR
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  清除中断寄存器
                        set the interrupt mask Enable/Disable appropriate Interrupt. ('1' : interrupt enable)
                        If any bit in IMR is set as '0' then there is not interrupt signal though the bit is
                         set in IR register.
*
* Parameter           :  addr ，a pointer to a 4 -byte array responsible to set the Gateway IP address.
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code
*****************************************************************************************************************************/

/**
@brief
*/
void clearIR(uint8_t mask)
{
  IINCHIP_WRITE(IR, ~mask | getIR() ); // must be setted 0x10.
}

/**
@brief  This sets the maximum segment size of TCP in Active Mode), while in Passive Mode this is set by peer
*/
void setSn_MSS(SOCKET s, uint16_t Sn_MSSR)
{
  IINCHIP_WRITE( Sn_MSSR0(s), (uint8_t)((Sn_MSSR & 0xff00) >> 8));
  IINCHIP_WRITE( Sn_MSSR1(s), (uint8_t)(Sn_MSSR & 0x00ff));
}
/*
void setSn_TTL(SOCKET s, uint8_t ttl)
{
   IINCHIP_WRITE( Sn_TTL(s) , ttl);
}

*/

/**
@brief  get socket interrupt status

These below functions are used to read the Interrupt & Soket Status register
*/
uint8_t getSn_IR(SOCKET s)
{
   return IINCHIP_READ(Sn_IR(s));
}


/**
@brief   get socket status
*/
uint8_t getSn_SR(SOCKET s)
{
   return IINCHIP_READ(Sn_SR(s));
}


/**
@brief  get socket TX free buf size

This gives free buffer size of transmit buffer. This is the data size that user can transmit.
User shuold check this value first and control the size of transmitting data
*/
uint16_t getSn_TX_FSR(SOCKET s)
{
  uint16_t val=0,val1=0;
  do
  {
    val1 = IINCHIP_READ(Sn_TX_FSR0(s));
    val1 = (val1 << 8) + IINCHIP_READ(Sn_TX_FSR1(s));
      if (val1 != 0)
    {
        val = IINCHIP_READ(Sn_TX_FSR0(s));
        val = (val << 8) + IINCHIP_READ(Sn_TX_FSR1(s));
    }
  } while (val != val1);
   return val;
}


/**
@brief   get socket RX recv buf size

This gives size of received data in receive buffer.
*/
uint16_t getSn_RX_RSR(SOCKET s)														// 获取空闲接收缓存寄存器的值
{
  uint16_t val=0,val1=0;
  do
  {
    val1 = IINCHIP_READ(Sn_RX_RSR0(s));									// MCU读Sn_RX_RSR的低8位，并赋给val1
    val1 = (val1 << 8) + IINCHIP_READ(Sn_RX_RSR1(s));		// 读高8位，并与低8位相加赋给val1
    if(val1 != 0)																				// 若Sn_RX_RSR的值不为0，将其赋给val
    {
        val = IINCHIP_READ(Sn_RX_RSR0(s));
        val = (val << 8) + IINCHIP_READ(Sn_RX_RSR1(s));
    }
  } while (val != val1);																// 判断val与val1是否相等，若不等，重新返回do循环，若相等，跳出循环
   return val;																					// 将val的值返回给getSn_RX_RSR
}


/**
@brief   This function is being called by send() and sendto() function also.

This function read the Tx write pointer register and after copy the data in buffer update the Tx write pointer
register. User should read upper byte first and lower byte later to get proper value.
*/
void send_data_processing(SOCKET s, uint8_t *data, uint16_t len)
{
  uint16_t ptr =0;
  uint32_t addrbsb =0;
  if(len == 0)
  {
    //printf("CH: %d Unexpected1 length 0\r\n", s);
    return;
  }


  ptr = IINCHIP_READ( Sn_TX_WR0(s) );
  ptr = ((ptr & 0x00ff) << 8) + IINCHIP_READ(Sn_TX_WR1(s));

  addrbsb = (uint32_t)(ptr<<8) + (s<<5) + 0x10;
  WriteW5500_Buf(addrbsb, data, len);

  ptr += len;
  IINCHIP_WRITE( Sn_TX_WR0(s) ,(uint8_t)((ptr & 0xff00) >> 8));
  IINCHIP_WRITE( Sn_TX_WR1(s),(uint8_t)(ptr & 0x00ff));
}

/**
@brief  This function is being called by recv() also.

This function read the Rx read pointer register
and after copy the data from receive buffer update the Rx write pointer register.
User should read upper byte first and lower byte later to get proper value.
*/
void recv_data_processing(SOCKET s, uint8_t *data, uint16_t len)
{
  uint16_t ptr = 0;
  uint32_t addrbsb = 0;

  if(len == 0)						// 若接收数据的长度为0，则串口打印“"CH: 0 Unexpected2 length 0”
  {
//    printf("CH: %d Unexpected2 length 0\r\n", s);
    return;
  }

	// MCU读取Sn_RX_RD接收写指针寄存器的值，并赋给ptr
	// Sn_RX_RD保存接收缓存中数据的首地址，若有数据接收，则接收完后该寄存器值要更新
  ptr = IINCHIP_READ( Sn_RX_RD0(s) );
  ptr = ((ptr & 0x00ff) << 8) + IINCHIP_READ( Sn_RX_RD1(s) );

  addrbsb = (uint32_t)(ptr<<8) + (s<<5) + 0x18;		// 获取接收到的数据的绝对地址
  ReadW5500_Buf(addrbsb, data, len);							// 通过绝对地址，将接收到的数据发给MCU

	// 更新Sn_RX_RD寄存器的值
	ptr += len;														//
  IINCHIP_WRITE( Sn_RX_RD0(s), (uint8_t)((ptr & 0xff00) >> 8));
  IINCHIP_WRITE( Sn_RX_RD1(s), (uint8_t)(ptr & 0x00ff));
}

void setSn_IR(uint8_t s, uint8_t val)
{
    IINCHIP_WRITE(Sn_IR(s), val);
}
/*****************************************************************************************************************************
* Function Name       :  W5500_GPIO_Init
* Create Date         :  2016/10/12
* Author/Corporation  :  YiPeng/Gaussian Robot
* Description         :  Initializ W5500 GPIO ;
*                        
* Parameter           :  speed,speed
* Return Code         :  None
* Global Variable     :  None
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code      
*****************************************************************************************************************************/

void W5500_GPIO_Init(void)
{

    GPIO_InitTypeDef  GPIO_InitStructure;
//  EXTI_InitTypeDef  EXTI_InitStructure; 

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA| RCC_AHB1Periph_GPIOB, ENABLE);   
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* W5500_RST引脚初始化配置 */
  GPIO_InitStructure.GPIO_Pin  = W5500_RST_PIN;
  GPIO_InitStructure.GPIO_Speed	= W5500_RST_SPEED;
  GPIO_InitStructure.GPIO_Mode = W5500_RST_MODE;
  GPIO_InitStructure.GPIO_OType = W5500_RST_OTYPE;
  GPIO_Init(W5500_RST_PORT, &GPIO_InitStructure);
  W5500_RST_ON;
  
  /* W5500_INT引脚初始化配置 */ 
  GPIO_InitStructure.GPIO_Pin = W5500_INT_PIN;
  GPIO_InitStructure.GPIO_Speed = W5500_INT_SPEED;
  GPIO_InitStructure.GPIO_Mode = W5500_INT_MODE;
  GPIO_InitStructure.GPIO_PuPd = W5500_INT_PUPD;
  GPIO_Init(W5500_INT_PORT, &GPIO_InitStructure);
    

  W5500_RST_OFF;
}
/****************************************************************************************************************************
* Function Name       :  HardResetW5500
* Create Date         :  2016/10/12
* Author/Corporation  :  YiPeng/Gaussian Robot
* Description         :  Hardware Reset W5500;
*                        
* Parameter           :  speed,speed
* Return Code         :  None
* Global Variable     :  None
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code      
*****************************************************************************************************************************/

void HardResetW5500(void)
{
  W5500_RST_ON;
  CycleCntDelayUs(1000);                             //手册上说是最小500us
  W5500_RST_OFF;
  CycleCntDelayMs(1600);
}

void HardResetW5500x(void)
{
  W5500_RST_ON;
  CycleCntDelayUs(1000);                             //手册上说是最小500us
  W5500_RST_OFF;
}
/****************************************************************************************************************************
* Function Name       :  SetNetwork
* Create Date         :  2016/10/12
* Author/Corporation  :  YiPeng/Gaussian Robot
* Description         :  设置IP，网关等等，初始化的值configMsg中;
*                        
* Parameter           :  speed,speed
* Return Code         :  None
* Global Variable     :  None
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code      
*****************************************************************************************************************************/
uint8_t IPbufx[4];
void SetNetwork(void)                           // 配置初始化IP信息并打印，初始化8个Socket
{  
  setSHAR(ConfigMsg.mac);
  setSUBR(ConfigMsg.sub);
  setGAR(ConfigMsg.gw);
  setSIPR(ConfigMsg.lip);

  sysinit(txsize, rxsize);                        // 初始化8个socket
  setRTR(2000);                                   // 设置超时时间
  setRCR(3);                                      // 设置最大重新发送次数
  getSIPR (IPbufx);
}


