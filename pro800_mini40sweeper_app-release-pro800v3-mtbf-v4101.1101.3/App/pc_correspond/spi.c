#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"
#include "socket.h"
#include "w5500.h"
#include "spi.h"

/*********************************SPI2********************************************/



#define SPI2_CS_PIN			GPIO_Pin_11	//定义W5500的CS引脚		
#define SPI2_CS_PORT		GPIOB
#define SPI2_CS_VALID		1
#define	SPI2_CS_PORT_RCC	RCC_AHB1Periph_GPIOB	

#define SPI2_SCK_PIN		GPIO_Pin_13	//定义W5500的SCK引脚		
#define SPI2_SCK_PORT		GPIOB
#define SPI2_SCK_VALID		1
#define	SPI2_SCK_PORT_RCC	RCC_AHB1Periph_GPIOB
#define SPI2_SCK_PIN_SOURCE GPIO_PinSource13

#define SPI2_MISO_PIN		GPIO_Pin_14	//定义W5500的MISO引脚		
#define SPI2_MISO_PORT		GPIOB
#define SPI2_MISO_VALID		1
#define	SPI2_MISO_PORT_RCC	RCC_AHB1Periph_GPIOB
#define SPI2_MISO_PIN_SOURCE GPIO_PinSource14

#define SPI2_MOSI_PIN		GPIO_Pin_15	//定义W5500的MOSI引脚		
#define SPI2_MOSI_PORT		GPIOB
#define SPI2_MOSI_VALID		1
#define	SPI2_MOSI_PORT_RCC	RCC_AHB1Periph_GPIOB
#define SPI2_MOSI_PIN_SOURCE GPIO_PinSource15

#if	SPI2_CS_VALID
	#define	SPI2_CS_ON			GPIO_SetBits(SPI2_CS_PORT, SPI2_CS_PIN);
	#define	SPI2_CS_OFF			GPIO_ResetBits(SPI2_CS_PORT, SPI2_CS_PIN);
#else
	#define	SPI2_CS_ON			GPIO_ResetBits(SPI2_CS_PORT, SPI2_CS_PIN);
	#define	SPI2_CS_OFF			GPIO_SetBits(SPI2_CS_PORT, SPI2_CS_PIN);
#endif

#if SPI2_SCK_VALID
	#define	SPI2_SCK_ON			GPIO_SetBits(SPI2_SCK_PORT, SPI2_SCK_PIN);
	#define	SPI2_SCK_OFF		GPIO_ResetBits(SPI2_SCK_PORT, SPI2_SCK_PIN);
#else
	#define	SPI2_SCK_ON			GPIO_ResetBits(SPI2_SCK_PORT, SPI2_SCK_PIN);
	#define	SPI2_SCK_OFF		GPIO_SetBits(SPI2_SCK_PORT, SPI2_SCK_PIN);
#endif

#if SPI2_MOSI_VALID
	#define	SPI2_MOSI_ON		GPIO_SetBits(SPI2_MOSI_PORT, SPI2_MOSI_PIN);
	#define	SPI2_MOSI_OFF		GPIO_ResetBits(SPI2_MOSI_PORT, SPI2_MOSI_PIN);
#else
	#define	SPI2_MOSI_ON		GPIO_ResetBits(SPI2_MOSI_PORT, SPI2_MOSI_PIN);
	#define	SPI2_MOSI_OFF		GPIO_SetBits(SPI2_MOSI_PORT, SPI2_MOSI_PIN);
#endif

#if SPI2_MISO_VALID
	#define	SPI2_MISO_ON		GPIO_SetBits(SPI2_MISO_PORT, SPI2_MISO_PIN);
	#define	SPI2_MISO_OFF		GPIO_ResetBits(SPI2_MISO_PORT, SPI2_MISO_PIN);
#else
	#define	SPI2_MISO_ON		GPIO_ResetBits(SPI2_MISO_PORT, SPI2_MISO_PIN);
	#define	SPI2_MISO_OFF		GPIO_SetBits(SPI2_MISO_PORT, SPI2_MISO_PIN);
#endif


void WIZ_SPI_Init(void)
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	SPI_InitTypeDef   	SPI_InitStructure;

	RCC_AHB1PeriphClockCmd(SPI2_CS_PORT_RCC | SPI2_SCK_PORT_RCC | SPI2_MISO_PORT_RCC | SPI2_MOSI_PORT_RCC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);


	/* 初始化SCK、MISO、MOSI引脚 */
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	
	GPIO_InitStructure.GPIO_Pin =  SPI2_SCK_PIN;
	GPIO_Init(SPI2_SCK_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  SPI2_MOSI_PIN;
	GPIO_Init(SPI2_MOSI_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  SPI2_MISO_PIN;
	GPIO_Init(SPI2_MISO_PORT, &GPIO_InitStructure);
	
	SPI2_SCK_ON;
	SPI2_MOSI_ON;
	SPI2_MISO_ON;
	
	GPIO_PinAFConfig(SPI2_SCK_PORT, SPI2_SCK_PIN_SOURCE, GPIO_AF_SPI2);  //?????????
  	GPIO_PinAFConfig(SPI2_MOSI_PORT, SPI2_MOSI_PIN_SOURCE, GPIO_AF_SPI2);
  	GPIO_PinAFConfig(SPI2_MISO_PORT, SPI2_MISO_PIN_SOURCE, GPIO_AF_SPI2);

	/* 初始化CS引脚 */
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = SPI2_CS_PIN;
	GPIO_Init(SPI2_CS_PORT, &GPIO_InitStructure);
	SPI2_CS_ON;

	/* 初始化配置STM32 SPI2 */
	SPI_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex;	//SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode=SPI_Mode_Master;							//设置为主SPI
	SPI_InitStructure.SPI_DataSize=SPI_DataSize_8b;						//SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL=SPI_CPOL_Low;							//时钟悬空低
	SPI_InitStructure.SPI_CPHA=SPI_CPHA_1Edge;							//数据捕获于第1个时钟沿
	SPI_InitStructure.SPI_NSS=SPI_NSS_Soft;								//NSS由外部管脚管理
	SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_2;	//波特率预分频值为2
	SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB;					//数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial=7;								//CRC多项式为7
	SPI_Init(SPI2,&SPI_InitStructure);									//根据SPI_InitStruct中指定的参数初始化外设SPI2寄存器
	SPI_Cmd(SPI2,ENABLE);	//STM32使能SPI1、2
}

// Connected to Data Flash
void WIZ_CS(uint8_t val)
{
	if (val == LOW)
	{
   		SPI2_CS_OFF;
	}
	else if (val == HIGH)
	{
   		SPI2_CS_ON;
	}
}


uint8_t SPI2_SendByte(uint8_t byte)
{
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI2, byte);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(SPI2);
}



