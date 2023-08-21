#ifndef __HC_CAN_H__
#define __HC_CAN_H__

#include "hc32_ll.h"
#include "objdict.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* CAN1相关引脚中断配置 */
#define CAN1_UNIT                        (CM_CAN1)
#define CAN1_PERIPH_CLK                  (FCG1_PERIPH_CAN1)
#define CAN1_TX_PORT                     (GPIO_PORT_D)
#define CAN1_TX_PIN                      (GPIO_PIN_01)
#define CAN1_TX_PIN_FUNC                 (GPIO_FUNC_60)
#define CAN1_RX_PORT                     (GPIO_PORT_D)
#define CAN1_RX_PIN                      (GPIO_PIN_00)
#define CAN1_RX_PIN_FUNC                 (GPIO_FUNC_61)
#define CAN1_CLK_UNIT                    (CLK_CAN1)
#define CAN1_CLK_SRC                     (CLK_CANCLK_SYSCLK_DIV6)
#define CAN1_INT_PRIO                    (DDL_IRQ_PRIO_05)
#define CAN1_INT_SRC                     (INT_SRC_CAN1_HOST)
#define CAN1_INT_IRQn                    (INT092_IRQn)

/* CAN2相关引脚中断配置 */
#define CAN2_UNIT                        (CM_CAN2)
#define CAN2_PERIPH_CLK                  (FCG1_PERIPH_CAN2)
#define CAN2_TX_PORT                     (GPIO_PORT_B)
#define CAN2_TX_PIN                      (GPIO_PIN_06)
#define CAN2_TX_PIN_FUNC                 (GPIO_FUNC_62)
#define CAN2_RX_PORT                     (GPIO_PORT_B)
#define CAN2_RX_PIN                      (GPIO_PIN_05)
#define CAN2_RX_PIN_FUNC                 (GPIO_FUNC_63)
#define CAN2_CLK_UNIT                    (CLK_CAN2)
#define CAN2_CLK_SRC                     (CLK_CANCLK_SYSCLK_DIV6)
#define CAN2_INT_PRIO                    (DDL_IRQ_PRIO_05)
#define CAN2_INT_SRC                     (INT_SRC_CAN2_HOST)
#define CAN2_INT_IRQn                    (INT093_IRQn)

/* CAN 中断类型选择 */
#define CAN_INT_SEL                     (CAN_INT_RX_BUF_WARN | \
                                         CAN_INT_RX_BUF_FULL | \
                                         CAN_INT_RX_OVERRUN  | \
                                         CAN_INT_RX)

/* 接收过滤器配置. */
#define CAN_FILTER_SEL                  (CAN_FILTER1 | CAN_FILTER2 | CAN_FILTER3)
#define CAN_FILTER_NUM                  (3U)

#define CAN_FILTER1_ID                  (0x000UL)
#define CAN_FILTER1_ID_MASK             (0x7FFUL)
#define CAN_FILTER1_ID_TYPE             (CAN_ID_STD)        /*接受帧ID=0x701的标准帧数据*/

#define CAN_FILTER2_ID                  (0x00000000UL)
#define CAN_FILTER2_ID_MASK             (0x1FFFFFFFUL)
#define CAN_FILTER2_ID_TYPE             (CAN_ID_EXT)        /*接受帧ID=0x121314x5的扩展帧数据*/

#define CAN_FILTER3_ID                  (0x1A1B1C1DUL)
#define CAN_FILTER3_ID_MASK             (0x0000000FUL)
#define CAN_FILTER3_ID_TYPE             (CAN_ID_STD_EXT)    /*接收扩展帧ID =0x1A1B1C1x,standard ID 0x41x(0x1A1B1C1D & 0x7FF(standard ID mask)). */

int32_t can_function_test(void);
/*初始化CAN1*/
void can1_config_init(void);
/*初始化CAN2*/
void can2_config_init(void);
/*发送CAN标准帧数据*/
int can_standard_send(CM_CAN_TypeDef *CANx, uint32_t id, void* f_data, uint8_t data_len);
/*发送CAN扩展帧数据，数据长度0~8*/
int can_extend_send(CM_CAN_TypeDef *CANx, uint32_t id, void* f_data, uint8_t data_len);
int CanWriteSDO(uint8_t f_id, uint16_t f_index, uint8_t f_sub_index, uint8_t f_data_type, void* f_data);
int CanReadSDO( uint8_t f_id, uint16_t f_index, uint8_t f_sub_index, uint8_t f_data_type, void* f_data );
int Can2ReadSDO( uint8_t f_id, uint16_t f_index, uint8_t f_sub_index, uint8_t f_data_type, void* f_data );
int Can2WriteSDO( uint8_t f_id, uint16_t f_index, uint8_t f_sub_index, uint8_t f_data_type, void* f_data );

extern QueueHandle_t bms_can2_rx_quene_handle;
extern QueueHandle_t anti_can2_rx_quene_handle;

#endif /* __HC_CAN_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
