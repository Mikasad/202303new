/**
  *****************************************************************************
  * @file    tcp_server.c
  * @author  李建
  * @version V1.0.0
  * @date    2018-09-04
  * @brief   tcp服务端的实现
  *****************************************************************************
  * @history
  *
  * 1. Date:2019-07-20
  *    Author:李建
  *    Modification:建立文件
  *
  *****************************************************************************
  */

//#include "stm32f4xx_hal.h"
//#include "lwip.h"
#include "tcp.h"
#include "tcp_server.h"
#include "nanopb_tcpip.h"
//#include "rtc_time.h"
//#include "my_debug.h"
//#include "info_store.h"
#include <string.h>

/* 定义端口号 */
#define TCP_LOCAL_PORT     5000 /* 本地端口 */
#define TCP_DEBUG_PORT     6000
#define TCP_ROUTER_PORT    6666

static uint8_t gs_rxBufSizeState = 0;//状态信息，当获取到的socket接收buf的size大于0时认为和pc以及w5500连接正常。
static struct tcp_pcb* m_debug_tpcb = NULL;

static char m_sys_stats_buff[2048] = {0};
struct tcp_pcb * server_tpcb = NULL;
struct pbuf * server_p = NULL;

/******************************************************************************
* 描述  : 接收回调函数，接收上位机数据，解析后回复
 * 参数  : -
 * 返回  : -
******************************************************************************/
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb,
                             struct pbuf *p, err_t err)
{    
 
	static uint32_t last_tick =0;
	server_tpcb = tpcb;
	server_p = p;
	
    if (p != NULL)
    {
        if(p->len > 0)
        {  
			if(HAL_GetTick() - last_tick>100) //用于测试
			{
				printf("tick:%d\r\n",HAL_GetTick());
			}
			last_tick = HAL_GetTick();
			if( gs_rxBufSizeState < 10 )
            {
                gs_rxBufSizeState++;
            }			
            if (p->len > NONOPD_BUF_LEN)
                g_nanopb_buf_len = NONOPD_BUF_LEN;
            else
                g_nanopb_buf_len = p->len;
            memcpy(nanopb_buf, ((uint8_t *)p->payload), g_nanopb_buf_len);
            uint8_t status = RunNanopb(nanopb_buf, &g_nanopb_buf_len, NONOPD_BUF_LEN);
			if(!status)			
			printf("s:%d\r\n",status);
			if(!status)
			{
				return ERR_OK;
			}
            tcp_write(tpcb, nanopb_buf, g_nanopb_buf_len, 1);//send(client_socket,(uint8_t *)&nanopb_buf,len, 1);
            tcp_output( tpcb );
            
            memset(nanopb_buf, 0, NONOPD_BUF_LEN); //发送完成后清空缓冲区

        }
        
        tcp_recved(tpcb, p->tot_len);
        
        /* 释放缓冲区数据 */
        pbuf_free(p);
    }
    else if (err == ERR_OK)
    {
        printf("tcp client closed\r\n");
        
        tcp_recved(tpcb, p->tot_len);
        server_tpcb = NULL;
        return tcp_close(tpcb);
    }

    return ERR_OK;
}

/******************************************************************************
 * 描述  : 客户端接入回调函数
 * 参数  : -
 * 返回  : -
******************************************************************************/
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
//    printf("tcp client connected\r\n");
//    
//    printf("ip %d:%d:%d:%d port:%d\r\n",
//        *((uint8_t *)&newpcb->remote_ip.addr),
//        *((uint8_t *)&newpcb->remote_ip.addr + 1),
//        *((uint8_t *)&newpcb->remote_ip.addr + 2),
//        *((uint8_t *)&newpcb->remote_ip.addr + 3),
//        newpcb->remote_port);
    
//   tcp_write(newpcb, "tcp client connected", strlen("tcp client connected"), 0);
    
    /* 注册接收回调函数 */
    tcp_recv(newpcb, tcp_server_recv);

    return ERR_OK;
}

/******************************************************************************
 * 描述  : 创建tcp服务器
 * 参数  : 无
 * 返回  : 无
******************************************************************************/
void tcp_server_init(void)
{
    struct tcp_pcb *tpcb;
//	static uint8_t tcp_server_existing = 0;

    /* 创建tcp控制块 */
    tpcb = tcp_new();

    if (tpcb != NULL)
    {
        err_t err;
        
        /* 绑定端口接收，接收对象为所有ip地址 */
        err = tcp_bind(tpcb, IP_ADDR_ANY, TCP_LOCAL_PORT);

        if (err == ERR_OK)
        {
            /* 监听 */
            tpcb = tcp_listen(tpcb);

            /* 注册接入回调函数 */
            tcp_accept(tpcb, tcp_server_accept);
            
            printf("tcp server listening\r\n");
        }
        else
        {
            memp_free(MEMP_TCP_PCB, tpcb);
            
            printf("can not bind pcb\r\n");
        }

    }
}

uint8_t GetRxBufSizeState(void)
{
	uint8_t ret = 0;
    if( gs_rxBufSizeState >= 10 )
    {
        ret = 1;
        gs_rxBufSizeState--;
    }
	return ret;
}

void SetRxBufSizeStatus(void)
{
    gs_rxBufSizeState = 0;
}
/*************************************************************************************************************
                                                路由端口
**************************************************************************************************************/
/******************************************************************************
 * 描述  : 接收回调函数
 * 参数  : -
 * 返回  : -
******************************************************************************/
//extern uint32_t g_last_router_tick;
//extern uint8_t g_router_communication_flag;
static err_t tcp_router_recv(void *arg, struct tcp_pcb *tpcb,
                             struct pbuf *p, err_t err)
{    
    /* 数据回传 */
    //tcp_write(tpcb, p->payload, p->len, 1);
    
    if (p != NULL)
    {
        if(p->len > 0)
        {
            if( NULL!=strstr(p->payload, "hello") )
            {
//                g_last_router_tick = xTaskGetTickCount();
//                g_router_communication_flag = 1;
            }
        }
        
        tcp_recved(tpcb, p->tot_len);
        
        /* 释放缓冲区数据 */
        pbuf_free(p);
    }
    else if (err == ERR_OK)
    {
        printf("router client closed\r\n");
        
        tcp_recved(tpcb, p->tot_len);
        
        return tcp_close(tpcb);
    }

    return ERR_OK;
}

/******************************************************************************
 * 描述  : 客户端接入回调函数
 * 参数  : -
 * 返回  : -
******************************************************************************/
static err_t tcp_router_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    printf("router client connected\r\n");
    
    printf("ip %d:%d:%d:%d port:%d\r\n",
        *((uint8_t *)&newpcb->remote_ip.addr),
        *((uint8_t *)&newpcb->remote_ip.addr + 1),
        *((uint8_t *)&newpcb->remote_ip.addr + 2),
        *((uint8_t *)&newpcb->remote_ip.addr + 3),
        newpcb->remote_port);
    
    //tcp_write(newpcb, "tcp client connected", strlen("tcp client connected"), 0);
    
    /* 注册接收回调函数 */
    tcp_recv(newpcb, tcp_router_recv);

    return ERR_OK;
}

/******************************************************************************
 * 描述  : 创建tcp服务器
 * 参数  : 无
 * 返回  : 无
******************************************************************************/
void tcp_router_init(void)
{
    struct tcp_pcb *tpcb;

    /* 创建tcp控制块 */
    tpcb = tcp_new();

    if (tpcb != NULL)
    {
        err_t err;
        
        /* 绑定端口接收，接收对象为所有ip地址 */
        err = tcp_bind(tpcb, IP_ADDR_ANY, TCP_ROUTER_PORT);

        if (err == ERR_OK)
        {
            /* 监听 */
            tpcb = tcp_listen(tpcb);

            /* 注册接入回调函数 */
            tcp_accept(tpcb, tcp_router_accept);
            
            printf("router server listening\r\n");
        }
        else
        {
            memp_free(MEMP_TCP_PCB, tpcb);
            
            printf("router not bind pcb\r\n");
        }

    }
}

/*************************************************************************************************************
                                                调试端口
**************************************************************************************************************/

/******************************************************************************
 * 描述  : 接收回调函数
 * 参数  : -
 * 返回  : -
******************************************************************************/
static err_t tcp_debug_recv(void *arg, struct tcp_pcb *tpcb,
                             struct pbuf *p, err_t err)
{    
    uint16_t len = 0;
    if (p != NULL)
    {
        if(p->len > 0)
        {
			if( m_debug_tpcb == NULL )
			{
                m_debug_tpcb = tpcb;
			}
            else if( m_debug_tpcb != tpcb )
            {
                tcp_abort( m_debug_tpcb );
                m_debug_tpcb = tpcb;
            }
            if(strstr( p->payload, "os list" ) != NULL )
            {
                tcp_write( tpcb, "TaskName\t\t\tStatus Priority Stack   Num\n", 39, 1 );
                tcp_write( tpcb, "---------------------------------------------------------------\n", 64, 1 );
#if (configUSE_TRACE_FACILITY && configUSE_STATS_FORMATTING_FUNCTIONS)
                memset( m_sys_stats_buff, 0, sizeof(m_sys_stats_buff));
                vTaskList( m_sys_stats_buff );
                len = strlen(m_sys_stats_buff);
//                m_sys_stats_buff[len] = '\n';
//                len++;
                tcp_write( tpcb, m_sys_stats_buff, len, 1 );
#endif
                tcp_write( tpcb, "---------------------------------------------------------------\n", 64, 1 );
                len = sprintf( m_sys_stats_buff, "B:Block,R:Ready,D:Delete,S:Stop,X:Running, SurplusStack:%u\n\n\n", xPortGetFreeHeapSize());
                tcp_write( tpcb, m_sys_stats_buff, len, 1 );
            }
        }
        memset( p->payload, 0, p->len );
        tcp_recved(tpcb, p->tot_len);
        
        /* 释放缓冲区数据 */
        pbuf_free(p);
    }
    else if (err == ERR_OK)
    {
        printf("debug client closed\r\n");
        
        tcp_recved(tpcb, p->tot_len);
        
        m_debug_tpcb = NULL;
        
//        g_log_show_enable = false;
//        DebugAbort();
        
        return tcp_close(tpcb);
    }

    return ERR_OK;
}

/******************************************************************************
 * 描述  : 客户端接入回调函数
 * 参数  : -
 * 返回  : -
******************************************************************************/
static err_t tcp_debug_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
//    printf("tcp client connected\r\n");
//    
//    printf("ip %d:%d:%d:%d port:%d\r\n",
//        *((uint8_t *)&newpcb->remote_ip.addr),
//        *((uint8_t *)&newpcb->remote_ip.addr + 1),
//        *((uint8_t *)&newpcb->remote_ip.addr + 2),
//        *((uint8_t *)&newpcb->remote_ip.addr + 3),
//        newpcb->remote_port);
    
    //tcp_write(newpcb, "tcp client connected", strlen("tcp client connected"), 0);
    
    /* 注册接收回调函数 */
    tcp_recv(newpcb, tcp_debug_recv);

    return ERR_OK;
}

/******************************************************************************
 * 描述  : 创建tcp服务器
 * 参数  : 无
 * 返回  : 无
******************************************************************************/

void tcp_debug_init(void)
{
    struct tcp_pcb *tpcb;

    /* 创建tcp控制块 */
    tpcb = tcp_new();

    if (tpcb != NULL)
    {
        err_t err;
        
        /* 绑定端口接收，接收对象为所有ip地址 */
        err = tcp_bind(tpcb, IP_ADDR_ANY, TCP_DEBUG_PORT);

        if (err == ERR_OK)
        {
            /* 监听 */
            tpcb = tcp_listen(tpcb);

            /* 注册接入回调函数 */
            tcp_accept(tpcb, tcp_debug_accept);
            
//            printf("debug server listening\r\n");
        }
        else
        {
            memp_free(MEMP_TCP_PCB, tpcb);
            
//            printf("debug can not bind pcb\r\n");
        }

    }
}

int tcp_debug_show( uint8_t* buff, uint16_t len )
{
    static TickType_t s_client_monitor_tick = 0;
    static uint8_t s_error_count = 0;
    err_t error;
    int ret = 0;
    if( m_debug_tpcb != NULL )
    {
        error = tcp_write( m_debug_tpcb, buff, len, 1 );
        
        if( ERR_OK == error )
        {
            tcp_output( m_debug_tpcb );
            s_error_count = 0;
        }
        else
        {
            if(!s_error_count)
                s_client_monitor_tick = xTaskGetTickCount();
            s_error_count++;
            
            if( s_error_count >= 10 )
            {
                s_error_count = 10;
                if( xTaskGetTickCount()-s_client_monitor_tick >= 2000 )
                {
                    s_error_count = 0;
//                    tcp_abort( m_debug_tpcb );
//                    m_debug_tpcb = NULL;
//                    g_log_show_enable = false;
//                    DebugAbort();
                    ret = -1;
                }
            }
        }
    }
    else
    {
        ret = 1;
    }
    return ret;
}



