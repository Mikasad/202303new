/*******************************************************************************************************************************
*File Name   : socket.h
*Copyright   : Copyright (C) 2016-2016 Gaussian Robot,All Ringhts Reserved
*
*CPU         : Stm32F407
*Create Date : 2016/10/15
*Author      : YiPeng
*Description : 
*--------------------------------------------------------
*No  Version  Date       Recised By  Item    Description
*1   V0.1     16.10.12   Yipeng              
*
*******************************************************************************************************************************/

#ifndef	_SOCKET_H_
#define	_SOCKET_H_
/******************************************************************************************************************************
*             Include File Section
*******************************************************************************************************************************/

#include "stm32f4xx.h"
#include "w5500.h"
/******************************************************************************************************************************
*             Prototype Declare Section
*******************************************************************************************************************************/

extern uint8_t socket(SOCKET s, uint8_t protocol, uint16_t port, uint8_t flag); // Opens a socket(TCP or UDP or IP_RAW mode)
extern void Close(SOCKET s); // Close socket
extern uint8_t Connect(SOCKET s, uint8_t * addr, uint16_t port); // Establish TCP connection (Active connection)
extern void Disconnect(SOCKET s); // disconnect the connection
extern uint8_t Listen(SOCKET s);	// Establish TCP connection (Passive connection)
extern uint16_t send(SOCKET s, const uint8_t * buf, uint16_t len); // Send data (TCP)
extern uint16_t recv(SOCKET s, uint8_t * buf, uint16_t len);	// Receive data (TCP)

#ifdef __MACRAW__
void macraw_open(void);
uint16_t macraw_send( const uint8_t * buf, uint16_t len ); //Send data (MACRAW)
uint16_t macraw_recv( uint8_t * buf, uint16_t len ); //Recv data (MACRAW)
#endif

#endif
/* _SOCKET_H_ */

