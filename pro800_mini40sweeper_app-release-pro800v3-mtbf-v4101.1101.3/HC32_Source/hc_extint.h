#ifndef __HC_EXIT_H__
#define __HC_EXIT_H__

#include "hc32_ll.h"

#define ECN0_PORT               (GPIO_PORT_A)
#define ECN0_PIN                (GPIO_PIN_15)
#define ECN0_EXTINT_CH          (EXTINT_CH15)
#define ECN0_INT_SRC            (INT_SRC_PORT_EIRQ15)
#define ECN0_INT_IRQn           (INT031_IRQn)
#define ECN0_INT_PRIO           (DDL_IRQ_PRIO_DEFAULT)

#define ECN1_PORT               (GPIO_PORT_E)
#define ECN1_PIN                (GPIO_PIN_00)
#define ECN1_EXTINT_CH          (EXTINT_CH00)
#define ECN1_INT_SRC            (INT_SRC_PORT_EIRQ0)
#define ECN1_INT_IRQn           (INT024_IRQn)
#define ECN1_INT_PRIO           (DDL_IRQ_PRIO_DEFAULT)
 
extern __IO uint32_t ENC0_Count,ENC1_Count ;

/*外部输入中断初始化*/
void HC_ExtiInit(void);
/*PA15外部输入中断初始化*/
void PA15_ExtiInit(void);
/*PA15外部输入中断初始化*/
void PE0_ExtiInit(void);






#endif 

