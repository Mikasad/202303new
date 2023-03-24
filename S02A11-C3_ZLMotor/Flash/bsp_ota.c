#include "bsp_ota.h"

pFunction Jump_To_Application;
uint32_t JumpAddress;

/*------------------------------------------------
Function:软件重启
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void SoftReset(void)
{
	__set_FAULTMASK(1);    
	NVIC_SystemReset();
}
/*------------------------------------------------
Function:跳转至bootloader
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
uint8_t Jump_to_APP(uint32_t APP_addr)
{
	if(((*(__IO uint32_t*)APP_addr) & 0x2FFE0000 ) == 0x20000000)
	{
		JumpAddress = *(__IO uint32_t*) (APP_addr + 4);
		Jump_To_Application = (pFunction) JumpAddress;
		__set_PRIMASK(1);
		HAL_NVIC_DisableIRQ(SysTick_IRQn);
//		__disable_irq();
		__set_MSP(*(__IO uint32_t*) APP_addr);
		Jump_To_Application();
	}
	else
		return 0;
	return 0;
}



