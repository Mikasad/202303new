#include "ads1115.h"
#include "sim_i2c.h"
#include "cmsis_os.h"

/**I2C2 GPIO Configuration    
 PF0     ------> I2C2_SDA
 PF1     ------> I2C2_SCL
*/

#define ADC_SDA_PORT GPIOF
#define ADC_SDA_PIN GPIO_PIN_0
#define ADC_SCL_PORT GPIOF
#define ADC_SCL_PIN GPIO_PIN_1

volatile uint8_t ads1115_res[5];

static void start(void)
{
	I2C_Start(ADC_SDA_PORT,ADC_SCL_PORT,ADC_SDA_PIN,ADC_SCL_PIN);
}
static void ack(void)
{
	I2C_Ack(ADC_SDA_PORT,ADC_SCL_PORT,ADC_SDA_PIN,ADC_SCL_PIN);
}
static void master_ack(void)
{
	I2C_MasterAck(ADC_SDA_PORT,ADC_SCL_PORT,ADC_SDA_PIN,ADC_SCL_PIN);
}
static void stop(void)
{
	I2C_Stop(ADC_SDA_PORT,ADC_SCL_PORT,ADC_SDA_PIN,ADC_SCL_PIN);
}

static void I2C_write_byte(uint8_t byte)
{
	I2C_SendByte(byte,ADC_SDA_PORT,ADC_SCL_PORT,ADC_SDA_PIN,ADC_SCL_PIN);
}

static uint8_t I2C_read_byte(void)
{
	return	I2C_ReceiveByte(ADC_SDA_PORT,ADC_SCL_PORT,ADC_SDA_PIN,ADC_SCL_PIN);
}

void ads1115_init(uint8_t ch)
{
	uint8_t i = 0;
	ads1115_res[0] = 0x90 ;//address,命令写
	ads1115_res[1] = 0x01;//point to config register
	ads1115_res[2] = 0xC2 | (ch << 4);//AIN0:0xC2 AIN1:0xD2
	ads1115_res[3] = 0xE3;//----设置配置寄存器的低八位0xE3
	start();
	for (i=0;i<4;i++)
	{
		I2C_write_byte(ads1115_res[i]);
		ack();
	}
	stop();
}
//----指向ADS1115指针寄存器用于准备读取数据
void point_register(void)
{
	uint8_t i;
	ads1115_res[0] = 0x90;
	ads1115_res[1] = 0x00;
	start();//----发送起始信号
	for (i=0;i<2;i++)
	{
		I2C_write_byte(ads1115_res[i]);
		ack();
	}
	stop();//---发送停止信号
}

uint16_t ads1115_read(void)
{
	uint8_t result_L,result_H;
	uint16_t result;
	start();
    I2C_write_byte(0x91);
	ack();

	result_H = I2C_read_byte();
	master_ack();
	result_L = I2C_read_byte();
	master_ack();
	stop();
	result = result_H << 8 | result_L;
//-----这里可以加入负压处理----
//----if(result>0x7ff)
//------result=~result+1;测量的负压从8000~ffff，即取反加1	
	
//	usart_putchar(result_H);
//	usart_putchar(result_L);
	return result;
}

int get_ads1115_data(uint8_t ch, uint16_t* fp_data)
{
    static TickType_t s_iic_tick = 0;
	int ret=0;
    
	if(!s_iic_tick)
    {
        ads1115_init(ch);
        s_iic_tick = xTaskGetTickCount();
    }
    
	if(xTaskGetTickCount()-s_iic_tick >= 5)
	{
		point_register();
		*fp_data = ads1115_read();
        ret = 1;
        s_iic_tick = 0;
    }
    
	return ret;
}

