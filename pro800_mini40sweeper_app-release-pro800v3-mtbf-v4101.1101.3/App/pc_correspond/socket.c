#include <stdio.h>
#include "socket.h"
#include "w5500.h"
#include "spi.h"

/****************************************************************************************************************************
*	Function Name       :  Connect
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  This Socket function initialize the channel in perticular mode, and set the port and wait for W5200 done it.
                         1 for sucess else 0.
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code      
*****************************************************************************************************************************/
uint8_t socket(SOCKET s, uint8_t protocol, uint16_t port, uint8_t flag)
{
   uint8_t ret;
   if (
        ((protocol&0x0F) == Sn_MR_TCP)    ||
        ((protocol&0x0F) == Sn_MR_UDP)    ||
        ((protocol&0x0F) == Sn_MR_IPRAW)  ||
        ((protocol&0x0F) == Sn_MR_MACRAW) ||
        ((protocol&0x0F) == Sn_MR_PPPOE)
      )
   {
      Close(s);
      IINCHIP_WRITE(Sn_MR(s) ,protocol | flag);
      if (port != 0) {
         IINCHIP_WRITE( Sn_PORT0(s) ,(uint8_t)((port & 0xff00) >> 8));
         IINCHIP_WRITE( Sn_PORT1(s) ,(uint8_t)(port & 0x00ff));
      } else {
         // if don't set the source port, set local_port number.
         IINCHIP_WRITE(Sn_PORT0(s) ,(uint8_t)((5000 & 0xff00) >> 8));  //默认设置为5000
         IINCHIP_WRITE(Sn_PORT1(s) ,(uint8_t)(5000 & 0x00ff));
      }
      IINCHIP_WRITE( Sn_CR(s) ,Sn_CR_OPEN); // run sockinit Sn_CR

      /* wait to process the command... */
      while( IINCHIP_READ(Sn_CR(s)) )
         ;
      /* ------- */
      ret = 1;
   }
   else
   {
      ret = 0;
   }
   return ret;
}

/****************************************************************************************************************************
*	Function Name       :  Connect
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  This function close the socket and parameter is "s" which represent the socket number
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code      
*****************************************************************************************************************************/
void Close(SOCKET s)
{

   IINCHIP_WRITE( Sn_CR(s) ,Sn_CR_CLOSE);

   /* wait to process the command... */
   while( IINCHIP_READ(Sn_CR(s) ) );
   /* ------- */
        /* all clear */
   IINCHIP_WRITE( Sn_IR(s) , 0xFF);
}

/****************************************************************************************************************************
*	Function Name       :  Connect
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  This function established  the connection for the channel in passive (server) mode. This function waits for the request from the peer.
                         1 for success else 0.
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code      
*****************************************************************************************************************************/

uint8_t Listen(SOCKET s)
{
   uint8_t ret;			// 定义一个监听标志位，若Sn_CR的LISTEN命令发送成功，其值为1，否则为0
   
	if (IINCHIP_READ( Sn_SR(s) ) == SOCK_INIT)		// 若Sn_SR处于初始化状态，进入循环
   {
      IINCHIP_WRITE( Sn_CR(s) ,Sn_CR_LISTEN);		// MCU配置W5500为监听状态

      while( IINCHIP_READ(Sn_CR(s) ) )					// 配置完成，Sn_CR自动清零
         ;
      ret = 1;																	// LISTEN命令发送成功，ret=1
   }
   else
   {
      ret = 0;																	// 否则，ret=0
   }
   return ret;
}

/****************************************************************************************************************************
*	Function Name       :  Connect
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  This function established  the connection for the channel in Active (client) mode.
                         This function waits for the untill the connection is established.
                         1 for success else 0.
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code      
*****************************************************************************************************************************/
uint8_t Connect(SOCKET s, uint8_t * addr, uint16_t port)
{
    uint8_t ret;
    if
        (
            ((addr[0] == 0xFF) && (addr[1] == 0xFF) && (addr[2] == 0xFF) && (addr[3] == 0xFF)) ||
            ((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||
            (port == 0x00)
        )
    {
      ret = 0;
    }
    else
    {
        ret = 1;
        // set destination IP
        IINCHIP_WRITE( Sn_DIPR0(s), addr[0]);
        IINCHIP_WRITE( Sn_DIPR1(s), addr[1]);
        IINCHIP_WRITE( Sn_DIPR2(s), addr[2]);
        IINCHIP_WRITE( Sn_DIPR3(s), addr[3]);
        IINCHIP_WRITE( Sn_DPORT0(s), (uint8_t)((port & 0xff00) >> 8));
        IINCHIP_WRITE( Sn_DPORT1(s), (uint8_t)(port & 0x00ff));
        IINCHIP_WRITE( Sn_CR(s) ,Sn_CR_CONNECT);
        /* wait for completion */
        while ( IINCHIP_READ(Sn_CR(s) ) ) ;

        while ( IINCHIP_READ(Sn_SR(s)) != SOCK_SYNSENT )
        {
            if(IINCHIP_READ(Sn_SR(s)) == SOCK_ESTABLISHED)
            {
                break;
            }
            if (getSn_IR(s) & Sn_IR_TIMEOUT)
            {
                IINCHIP_WRITE(Sn_IR(s), (Sn_IR_TIMEOUT));  // clear TIMEOUT Interrupt
                ret = 0;
                break;
            }
        }
    }

   return ret;
}

/****************************************************************************************************************************
*	Function Name       :  Disconnect
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  This function used for disconnect the socket and parameter is "s" which represent the socket number
                         1 for success else 0.
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code      
*****************************************************************************************************************************/
void Disconnect(SOCKET s)
{
   IINCHIP_WRITE( Sn_CR(s) ,Sn_CR_DISCON);

   /* wait to process the command... */
   while( IINCHIP_READ(Sn_CR(s) ) )
      ;
   /* ------- */
}

/****************************************************************************************************************************
*	Function Name       :  send
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  This function used to send the data in TCP mode
*                        1 for success else 0.
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code      
*****************************************************************************************************************************/
uint16_t send(SOCKET s, const uint8_t * buf, uint16_t len)
{
  uint8_t status=0;
  uint16_t ret=0;
  uint16_t freesize=0;

  if (len > getIINCHIP_TxMAX(s)) ret = getIINCHIP_TxMAX(s);
  else ret = len;

  do
  {
    freesize = getSn_TX_FSR(s);
    status = IINCHIP_READ(Sn_SR(s));
    if ((status != SOCK_ESTABLISHED) && (status != SOCK_CLOSE_WAIT))
    {
      ret = 0;
      break;
    }
  } while (freesize < ret);

  send_data_processing(s, (uint8_t *)buf, ret);
  IINCHIP_WRITE( Sn_CR(s) ,Sn_CR_SEND);

  while( IINCHIP_READ(Sn_CR(s) ) );

  while ( (IINCHIP_READ(Sn_IR(s) ) & Sn_IR_SEND_OK) != Sn_IR_SEND_OK )
  {
    status = IINCHIP_READ(Sn_SR(s));
    if ((status != SOCK_ESTABLISHED) && (status != SOCK_CLOSE_WAIT) )
    {
//      printf("SEND_OK Problem!!\r\n");
      Close(s);
      return 0;
    }
  }
  IINCHIP_WRITE( Sn_IR(s) , Sn_IR_SEND_OK);

#ifdef __DEF_IINCHIP_INT__
   putISR(s, getISR(s) & (~Sn_IR_SEND_OK));
#else
   IINCHIP_WRITE( Sn_IR(s) , Sn_IR_SEND_OK);
#endif

   return ret;
}


/****************************************************************************************************************************
*	Function Name       :  send
*	Create Date         :  2016/10/12
*	Author/Corporation  :  YiPeng/Gaussian Robot
*	Description         :  This function is an application I/F function which is used to receive the data in TCP mode.
*                        It continues to wait for data as much as the application wants to receive.
*                        received data size for success else -1.
*-------Revision History------------------------------
* No.      Date       Revised By   Item    Description
* 1        16.10.12   YiPeng               first code      
*****************************************************************************************************************************/
uint16_t recv(SOCKET s, uint8_t * buf, uint16_t len)
{
   uint16_t ret=0;
   if ( len > 0 )
   {
      recv_data_processing(s, buf, len);				// 数据接收进程：将通过Sockets的buf接受的长度为len的数据写入指针对应的MCU的缓存地址

			IINCHIP_WRITE( Sn_CR(s) ,Sn_CR_RECV);			// MCU配置Sn_CR为RECV

      while( IINCHIP_READ(Sn_CR(s) ));					// 配置完成，Sn_CR自动清零

      ret = len;																// 将接收数据长度值赋给ret
   }
   return ret;																	// 返回ret的值。有返回值说明W5500有数据接收，并不断重复接收这一进程
}





