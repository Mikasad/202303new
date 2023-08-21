#ifndef TCP_SERVER_H
#define TCP_SERVER_H
#include <stdint.h>
#include <stdbool.h>

#include "nanopb_tcpip.h"
#include <string.h>
#include "main.h"
#include "tcp.h"

void tcp_server_init(void);
uint8_t GetRxBufSizeState(void);
void SetRxBufSizeStatus(void);

void tcp_debug_init(void);
int tcp_debug_show( uint8_t* buff, uint16_t len );

void tcp_router_init(void);
extern struct tcp_pcb * server_tpcb;
extern struct pbuf * server_p ;

#endif
