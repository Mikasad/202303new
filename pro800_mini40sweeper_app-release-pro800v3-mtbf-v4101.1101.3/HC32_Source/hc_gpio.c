/********************************************************************************
*文件名			: adc.c
*简  介			: adc配置，由官方测试例程修改获得。
*日	 期		: 2022/6/11
*作  者			: 井小飞  
********************************************************************************
*备注： 
*外部晶振为25MHz
*
********************************************************************************/
 
/******************************头文件*******************************************/
#include "hc_gpio.h"

/*****************************私有变量***************************************/

/******************************宏定义***************************************/
void HAL_GPIO_WritePin( uint32_t port ,uint32_t pin  ,uint32_t PinState  )
{
	if(PinState)
	{
		GPIO_SetPins(port, pin) ;
	}
	else
	{
		GPIO_ResetPins(port, pin) ;
	}
}

/***************************私有函数原形*************************************/

/****************************************************************************
*函数名	: port_init
*介	 绍	：GPIO初始化
*形  参 : 无
*返回值 : 无
******************************************************************************/
static void port_init(uint8_t u8Port, uint16_t u16Pin, uint16_t pull, uint16_t dir)
{
    stc_gpio_init_t stcGpioInit;

    (void)GPIO_StructInit(&stcGpioInit);
	  
//    stcGpioInit.u16PinState = PIN_STAT_RST;
    stcGpioInit.u16PinDir   = dir;
	  stcGpioInit.u16PullUp   = pull;
    (void)GPIO_Init(u8Port, u16Pin, &stcGpioInit);
}

/****************************************************************************
*函数名	: hc_gpio_init
*介	 绍	：GPIO端口初始化
*形  参 : 无
*返回值 : 无
******************************************************************************/
void hc_gpio_init(void)
{
	/*Configure GPIO pins : PEPin PEPin PEPin PEPin 
					   PEPin PEPin */
	port_init(GPIO_PORT_E, BRK1_Pin|ON_RELAY1_Pin|ON_RELAY3_Pin, PIN_PU_ON, PIN_DIR_OUT);
	port_init(GPIO_PORT_F, IN12_Pin|IN13_Pin|IN14_Pin|IN15_Pin, PIN_PU_ON, PIN_DIR_IN);
	port_init(GPIO_PORT_H, TLY_RST_Pin|SYS_LED_Pin, PIN_PU_ON, PIN_DIR_OUT);
	port_init(RS485_CONTROL1_GPIO_Port, RS485_CONTROL1_Pin, PIN_PU_ON, PIN_DIR_OUT);
	port_init(RS485_CONTROL2_GPIO_Port, RS485_CONTROL2_Pin, PIN_PU_ON, PIN_DIR_OUT);
	port_init(RS485_CONTROL3_GPIO_Port, RS485_CONTROL3_Pin, PIN_PU_ON, PIN_DIR_OUT);
	port_init(GPIO_PORT_H, HARD_VER1_Pin|IN6_Pin|IN0_Pin|DIN3_Pin|DIN4_Pin, PIN_PU_ON, PIN_DIR_IN);
//	port_init(GPIO_PORT_E, DIN2_Pin, PIN_PU_ON, PIN_DIR_IN);
//	port_init(GPIO_PORT_B, DIN1_Pin|STOP_Pin, PIN_PU_ON, PIN_DIR_IN);
	port_init(GPIO_PORT_B, STOP_Pin, PIN_PU_ON, PIN_DIR_IN);
	port_init(ALM_1_GPIO_Port, ALM_1_Pin, PIN_PU_ON, PIN_DIR_IN);
	port_init(GPIO_PORT_F, DIR0_Pin|BRK0_Pin, PIN_PU_ON, PIN_DIR_OUT);
	port_init(GPIO_PORT_G, DIR1_Pin|MOTOR_CTR_Pin|CO24V_2_Pin|CO24V_3_Pin|CO24V_4_Pin|
						CHARGE_SWITCH_Pin|DC5V_EN1_Pin|BUZZER_Pin, PIN_PU_ON, PIN_DIR_OUT);
	port_init(GPIO_PORT_G, MCU_KEY_Pin|IN2_Pin|IN1_Pin, PIN_PU_ON, PIN_DIR_IN);
	port_init(GPIO_PORT_E, ON_RELAY2_Pin|ON_RELAY4_Pin|CO24V_5_Pin|CO24V_6_Pin, 
						PIN_PU_OFF, PIN_DIR_OUT);
	port_init(GPIO_PORT_D, IN3_Pin|IN4_Pin|IN8_Pin|IN9_Pin 
						|IN10_Pin|IN11_Pin, PIN_PU_ON, PIN_DIR_IN);
	port_init(POWER_UP_GPIO_Port, POWER_UP_Pin, PIN_PU_OFF, PIN_DIR_OUT); //STM32是下拉，HC32下拉宏定义找不到
	port_init(IN5_GPIO_Port, IN5_Pin, PIN_PU_ON, PIN_DIR_IN);
	port_init(CO24V_1_GPIO_Port, CO24V_1_Pin, PIN_PU_ON, PIN_DIR_OUT); //H
	port_init(ALM_0_GPIO_Port, ALM_0_Pin, PIN_PU_OFF, PIN_DIR_IN);
	port_init(IN7_GPIO_Port, IN7_Pin, PIN_PU_OFF, PIN_DIR_IN);
	port_init(HARD_VER2_GPIO_Port, HARD_VER2_Pin, PIN_PU_ON, PIN_DIR_IN);
	port_init(GPIO_PORT_D, OUTPUT1_Pin|OUTPUT2_Pin, PIN_PU_OFF, PIN_DIR_OUT);
	port_init(GPIO_PORT_I, V3V3_EN_1_Pin|DC3V3_EN_Pin|V12_EN_Pin, PIN_PU_ON, PIN_DIR_OUT);
	/*Configure GPIO pin Output Level */
	GPIO_SetPins(GPIO_PORT_I, DC5V_EN_1_Pin);
	GPIO_SetPins(GPIO_PORT_I, DC3V3_EN_Pin|V12_EN_Pin|V3V3_EN_1_Pin);
	port_init(DC5V_EN_GPIO_Port, DC5V_EN_Pin, PIN_PU_ON, PIN_DIR_OUT);
	/*Configure GPIO pin Output Level */
	GPIO_SetPins(DC5V_EN_GPIO_Port, DC5V_EN_Pin);
	/*Configure GPIO pin Output Level */
	GPIO_ResetPins(GPIO_PORT_E, BRK1_Pin|ON_RELAY1_Pin
							 |ON_RELAY2_Pin|ON_RELAY3_Pin|ON_RELAY4_Pin);
	GPIO_SetPins(GPIO_PORT_E, CO24V_5_Pin|CO24V_6_Pin);



	/*Configure GPIO pin Output Level */
	GPIO_SetPins(TLY_RST_GPIO_Port, TLY_RST_Pin);

	/*Configure GPIO pin Output Level */
	GPIO_SetPins(RS485_CONTROL1_GPIO_Port, RS485_CONTROL1_Pin);

	/*Configure GPIO pin Output Level */
	GPIO_SetPins(RS485_CONTROL2_GPIO_Port, RS485_CONTROL2_Pin);

	/*Configure GPIO pin Output Level */
	GPIO_SetPins(RS485_CONTROL3_GPIO_Port, RS485_CONTROL3_Pin);

	/*Configure GPIO pin Output Level */
	GPIO_ResetPins(GPIO_PORT_F, DIR0_Pin|BRK0_Pin|CO24V_7_Pin);

	/*Configure GPIO pin Output Level */
	GPIO_ResetPins(GPIO_PORT_G, DIR1_Pin|MOTOR_CTR_Pin|
								 CHARGE_SWITCH_Pin|BUZZER_Pin);
	GPIO_SetPins(GPIO_PORT_G, CO24V_3_Pin|CO24V_2_Pin|
							 CO24V_4_Pin|DC5V_EN1_Pin|POWER_UP_Pin);

	/*Configure GPIO pin Output Level */
	GPIO_ResetPins(CO24V_1_GPIO_Port, CO24V_1_Pin);

	/*Configure GPIO pin Output Level */
	GPIO_ResetPins(GPIO_PORT_D, OUTPUT1_Pin|OUTPUT2_Pin);


}

/****************************************************************************
*函数名	: hc_gpio_output_low_test
*介	 绍	：GPIO管脚输出低电平测试
*形  参 : 无
*返回值 : 无
******************************************************************************/
static void hc_gpio_output_low_test(void)
{
	BUZZER_RESET();
	RELAY1_RESET();
	RELAY2_RESET();
	RELAY3_RESET();
	RELAY4_RESET();
	DC24V_OUT3_GND_RESET();
	DC24V_OUT4_GND_RESET();
	DC24V_OUT1_RESET();
	DC24V_OUT2_RESET();
	DC24V_OUT3_RESET();
	DC24V_OUT4_RESET();
	LED_RESET();
	OUT1_IO_RESET();
	OUT2_IO_RESET();
	V12_EN_RESET();
	DC5V_EN_1_RESET();
	DC3V3_EN_1_RESET();
	DC3V3_EN_RESET();
}
 
/****************************************************************************
*函数名	: hc_gpio_output_high_test
*介	 绍	：GPIO输出高电平测试
*形  参 : 无
*返回值 : 无
******************************************************************************/
static void hc_gpio_output_high_test(void)
{
	BUZZER_SET();
	RELAY1_SET();
	RELAY2_SET();
	RELAY3_SET();
	RELAY4_SET();
	DC24V_OUT3_GND_SET();
	DC24V_OUT4_GND_SET();
	DC24V_OUT1_SET();
	DC24V_OUT2_SET();
	DC24V_OUT3_SET();
	DC24V_OUT4_SET();
	LED_SET();
	OUT1_IO_SET();
	OUT2_IO_SET();
	V12_EN_SET();
	DC5V_EN_1_SET();
	DC3V3_EN_1_SET();
	DC3V3_EN_SET();
}

/****************************************************************************
*函数名	: hc_gpio_read_test
*介	 绍	：GPIO输入测试
*形  参 : 无
*返回值 : 无
******************************************************************************/
static en_pin_state_t hc_gpio_read_test(void)
{
//    return DIN1_IO_READ();
//	  return DIN2_IO_READ();
//	  return IN10_READ();
//	  return IN11_READ();
//	  return IN12_READ();
	return STOP_READ();
}
 
/****************************************************************************
*函数名	: hc_gpio_test
*介	 绍	：GPIO测试例程
*形  参 : 无
*返回值 : 无
******************************************************************************/
void hc_gpio_test(void)
{
	/* Register write enable for some required peripherals. */
	//LL_PERIPH_WE(LL_PERIPH_GPIO);
	/* GPIO initialize */
	hc_gpio_init();
	/* Register write protected for some required peripherals. */
	//LL_PERIPH_WP(LL_PERIPH_GPIO);
	for (;;) {
		hc_gpio_output_low_test();
		DDL_DelayMS(DLY_MS);

		hc_gpio_output_high_test();
		DDL_DelayMS(DLY_MS);

		if (DIN1_IO_READ() == PIN_SET)
			LED_SET();
		else
			LED_RESET();
	}
}


