#ifndef _HC_USART_H
#define _HC_USART_H

#include "hc32_ll.h"
#include "ring_buf.h"

/* Peripheral register WE/WP selection */
#define LL_PERIPH_SEL                   (LL_PERIPH_GPIO | LL_PERIPH_FCG | LL_PERIPH_PWC_CLK_RMU | \
                                         LL_PERIPH_EFM | LL_PERIPH_SRAM)

/* Application frame length max definition */
#define UART1_RX_BUF_SIZE 100
#define UART2_RX_BUF_SIZE 100
#define UART3_RX_BUF_SIZE 100
#define UART4_RX_BUF_SIZE 100
#define UART5_RX_BUF_SIZE 100
#define UART6_RX_BUF_SIZE 100

//-----------------------------------------------------------------------------//

/* USART RX/TX pin definition */
#define RS485_2_USART_RX_PORT                   (GPIO_PORT_D)   /* PD02: USART5_RX */
#define RS485_2_USART_RX_PIN                    (GPIO_PIN_02)
#define RS485_2_USART_RX_GPIO_FUNC              (GPIO_FUNC_20)

#define RS485_2_USART_TX_PORT                   (GPIO_PORT_C)   /* PC12: USART5_TX */
#define RS485_2_USART_TX_PIN                    (GPIO_PIN_12)
#define RS485_2_USART_TX_GPIO_FUNC              (GPIO_FUNC_20)

/* USART unit definition */
#define RS485_2_USART_UNIT                      (CM_USART5)
#define RS485_2_USART_FCG_ENABLE()              (FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_USART5, ENABLE))

/* USART interrupt definition */
#define RS485_2_USART_RX_ERR_IRQn               (INT016_IRQn)
#define RS485_2_USART_RX_ERR_INT_SRC            (INT_SRC_USART5_EI)

#define RS485_2_USART_RX_FULL_IRQn              (INT017_IRQn)
#define RS485_2_USART_RX_FULL_INT_SRC           (INT_SRC_USART5_RI)

#define RS485_2_USART_TX_EMPTY_IRQn             (INT018_IRQn)
#define RS485_2_USART_TX_EMPTY_INT_SRC          (INT_SRC_USART5_TI)

#define RS485_2_USART_TX_CPLT_IRQn              (INT019_IRQn)
#define RS485_2_USART_TX_CPLT_INT_SRC           (INT_SRC_USART5_TCI)

/* Ring buffer size */
#define RS485_2_RING_BUF_SIZE                   (100UL)

//-----------------------------------------------------------------------------//

/* USART RX/TX pin definition */
#define RS485_3_USART_RX_PORT                   (GPIO_PORT_D)   /* PD09: USART3_RX */
#define RS485_3_USART_RX_PIN                    (GPIO_PIN_09)
#define RS485_3_USART_RX_GPIO_FUNC              (GPIO_FUNC_37)

#define RS485_3_USART_TX_PORT                   (GPIO_PORT_D)   /* PD08: USART3_TX */
#define RS485_3_USART_TX_PIN                    (GPIO_PIN_08)
#define RS485_3_USART_TX_GPIO_FUNC              (GPIO_FUNC_36)

/* USART unit definition */
#define RS485_3_USART_UNIT                      (CM_USART3)
#define RS485_3_USART_FCG_ENABLE()              (FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_USART3, ENABLE))

/* USART interrupt definition */
#define RS485_3_USART_RX_ERR_IRQn               (INT097_IRQn)
#define RS485_3_USART_RX_ERR_INT_SRC            (INT_SRC_USART3_EI)

#define RS485_3_USART_RX_FULL_IRQn              (INT009_IRQn)
#define RS485_3_USART_RX_FULL_INT_SRC           (INT_SRC_USART3_RI)

#define RS485_3_USART_TX_EMPTY_IRQn             (INT010_IRQn)
#define RS485_3_USART_TX_EMPTY_INT_SRC          (INT_SRC_USART3_TI)

#define RS485_3_USART_TX_CPLT_IRQn              (INT011_IRQn)
#define RS485_3_USART_TX_CPLT_INT_SRC           (INT_SRC_USART3_TCI)

/* Ring buffer size */
#define RS485_3_RING_BUF_SIZE                   (100UL)

//-----------------------------------------------------------------------------//

/* USART RX/TX pin definition */
#define RS232_1_USART_RX_PORT                   (GPIO_PORT_C)   /* PC11: USART4_RX */
#define RS232_1_USART_RX_PIN                    (GPIO_PIN_11)
#define RS232_1_USART_RX_GPIO_FUNC              (GPIO_FUNC_20)

#define RS232_1_USART_TX_PORT                   (GPIO_PORT_C)   /* PC10: USART4_TX */
#define RS232_1_USART_TX_PIN                    (GPIO_PIN_10)
#define RS232_1_USART_TX_GPIO_FUNC              (GPIO_FUNC_20)

/* USART unit definition */
#define RS232_1_USART_UNIT                      (CM_USART4)
#define RS232_1_USART_FCG_ENABLE()              (FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_USART4, ENABLE))

/* USART interrupt definition */
#define RS232_1_USART_RX_ERR_IRQn               (INT012_IRQn)
#define RS232_1_USART_RX_ERR_INT_SRC            (INT_SRC_USART4_EI)

#define RS232_1_USART_RX_FULL_IRQn              (INT013_IRQn)
#define RS232_1_USART_RX_FULL_INT_SRC           (INT_SRC_USART4_RI)

#define RS232_1_USART_TX_EMPTY_IRQn             (INT014_IRQn)
#define RS232_1_USART_TX_EMPTY_INT_SRC          (INT_SRC_USART4_TI)

#define RS232_1_USART_TX_CPLT_IRQn              (INT015_IRQn)
#define RS232_1_USART_TX_CPLT_INT_SRC           (INT_SRC_USART4_TCI)

/* Ring buffer size */
#define RS232_1_RING_BUF_SIZE                   (100UL)

//-----------------------------------------------------------------------------//

/* USART RX/TX pin definition */
#define RS232_2_USART_RX_PORT                   (GPIO_PORT_G)   /* PG09: USART6_RX */
#define RS232_2_USART_RX_PIN                    (GPIO_PIN_09)
#define RS232_2_USART_RX_GPIO_FUNC              (GPIO_FUNC_37)

#define RS232_2_USART_TX_PORT                   (GPIO_PORT_G)   /* PG14: USART6_TX */
#define RS232_2_USART_TX_PIN                    (GPIO_PIN_14)
#define RS232_2_USART_TX_GPIO_FUNC              (GPIO_FUNC_36)

/* USART unit definition */
#define RS232_2_USART_UNIT                      (CM_USART6)
#define RS232_2_USART_FCG_ENABLE()              (FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_USART6, ENABLE))

/* USART interrupt definition */
#define RS232_2_USART_RX_ERR_IRQn               (INT020_IRQn)
#define RS232_2_USART_RX_ERR_INT_SRC            (INT_SRC_USART6_EI)

#define RS232_2_USART_RX_FULL_IRQn              (INT021_IRQn)
#define RS232_2_USART_RX_FULL_INT_SRC           (INT_SRC_USART6_RI)

#define RS232_2_USART_TX_EMPTY_IRQn             (INT022_IRQn)
#define RS232_2_USART_TX_EMPTY_INT_SRC          (INT_SRC_USART6_TI)

#define RS232_2_USART_TX_CPLT_IRQn              (INT023_IRQn)
#define RS232_2_USART_TX_CPLT_INT_SRC           (INT_SRC_USART6_TCI)

/* Ring buffer size */
#define RS232_2_RING_BUF_SIZE                   (100UL)


extern __IO en_flag_status_t m_recv_flag;
extern __IO uint16_t m_recv_len;
extern uint8_t g_uart2RxBuf[UART2_RX_BUF_SIZE];
extern uint8_t g_uart3RxBuf[UART3_RX_BUF_SIZE];
extern uint8_t g_uart4RxBuf[UART4_RX_BUF_SIZE];
extern stc_ring_buf_t m_stcRingBuf_RS485_3;
extern stc_ring_buf_t m_stcRingBuf_RS232_1;


void hc_usart2_init(void);
void hc_usart3_init(uint32_t baud_rate);
void hc_usart4_init(void);
void hc_usart5_init(void);
void hc_usart6_init(void);

void hc_usart2_transmit(const __IO uint8_t *u16TxString, const uint32_t len);
void hc_usart3_transmit(const __IO uint8_t *u16TxString, const uint32_t len);
void hc_usart4_transmit(const __IO uint8_t *u16TxString, const uint32_t len);
void hc_usart5_transmit(const __IO uint8_t *u16TxString, const uint32_t len);
void hc_usart6_transmit(const __IO uint8_t *u16TxString, const uint32_t len);

void hc_usart3_test(void);
void hc_usart4_test(void);
void hc_usart5_test(void);
void hc_usart6_test(void);

extern stc_ring_buf_t m_stcRingBuf_RS485_1;

extern uint8_t g_uart5RxBuf[UART5_RX_BUF_SIZE];
extern uint8_t m_au8DataBuf_RS485_2[RS485_2_RING_BUF_SIZE];
extern stc_ring_buf_t m_stcRingBuf_RS485_2;

extern uint8_t g_uart6RxBuf[UART6_RX_BUF_SIZE];
extern uint8_t m_au8DataBuf_RS232_2[RS232_2_RING_BUF_SIZE];
extern stc_ring_buf_t m_stcRingBuf_RS232_2;

#endif  
