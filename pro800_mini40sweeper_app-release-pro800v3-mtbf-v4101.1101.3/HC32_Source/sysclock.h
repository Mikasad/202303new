#ifndef __BSP_SYSCLOCK_H__
#define __BSP_SYSCLOCK_H__

#include "hc32_ll.h"



/*系统时钟初始化*/
void sysclock_init(void);
/*系统时钟切换测试*/
void sysclock_test(void);
void BSP_CLK_Init(void);
#endif  
