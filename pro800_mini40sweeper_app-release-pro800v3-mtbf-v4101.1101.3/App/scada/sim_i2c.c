#include "sim_i2c.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//volatile uint8_t io_index = 0;              
#define SCL_H(GPIOx,gpio_pin)         HAL_GPIO_WritePin( GPIOx, gpio_pin, GPIO_PIN_SET )
#define SCL_L(GPIOx,gpio_pin)         HAL_GPIO_WritePin( GPIOx, gpio_pin, GPIO_PIN_RESET )
   
#define SDA_H(GPIOx,gpio_pin)         HAL_GPIO_WritePin( GPIOx, gpio_pin, GPIO_PIN_SET )
#define SDA_L(GPIOx,gpio_pin)         HAL_GPIO_WritePin( GPIOx, gpio_pin, GPIO_PIN_RESET )

#define SCL_read(GPIOx,gpio_pin)      HAL_GPIO_ReadPin( GPIOx, gpio_pin)
#define SDA_read(GPIOx,gpio_pin)      HAL_GPIO_ReadPin( GPIOx, gpio_pin)


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/

/**/

void i2c_io_config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */
  
    __HAL_RCC_GPIOF_CLK_ENABLE();
    /**I2C2 GPIO Configuration    
    PF0     ------> I2C2_SDA
    PF1     ------> I2C2_SCL 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
}

void I2C_delay(void)
{       
   uint16_t i=160; 
   while(i)
   {
     i--;
   }
}

void I2C_Start(GPIO_TypeDef* sda_port ,GPIO_TypeDef* scl_port ,uint16_t sda_pin ,uint16_t scl_pin)
{
        SDA_H(sda_port,sda_pin) ;
        I2C_delay();
        SCL_H(scl_port,scl_pin) ;
        I2C_delay();
        SDA_L(sda_port,sda_pin)  ;
        I2C_delay();
        SCL_L(scl_port,scl_pin)  ;
        I2C_delay();

}

void I2C_Stop(GPIO_TypeDef* sda_port ,GPIO_TypeDef* scl_port ,uint16_t sda_pin ,uint16_t scl_pin)
{
        SCL_L(scl_port,scl_pin);
        SDA_L(sda_port,sda_pin)  ;
        I2C_delay();
        SCL_H(scl_port,scl_pin) ;
        I2C_delay();
        SDA_H(sda_port,sda_pin) ;
        I2C_delay();
}

void I2C_Ack(GPIO_TypeDef* sda_port ,GPIO_TypeDef* scl_port ,uint16_t sda_pin ,uint16_t scl_pin)
{       
        uint16_t i = 0;

        SCL_H(scl_port,scl_pin);
        I2C_delay();
        while(SDA_read(sda_port,sda_pin) && (i<280))
          i++;
         SCL_L(scl_port,scl_pin);
        I2C_delay();
}

void I2C_NoAck(GPIO_TypeDef* sda_port ,GPIO_TypeDef* scl_port ,uint16_t sda_pin ,uint16_t scl_pin)
{       

        SDA_H(sda_port,sda_pin) ;
        I2C_delay();
        SCL_H(scl_port,scl_pin) ;
        I2C_delay();
         SCL_L(scl_port,scl_pin);
        I2C_delay();
}

void I2C_MasterAck(GPIO_TypeDef* sda_port ,GPIO_TypeDef* scl_port ,uint16_t sda_pin ,uint16_t scl_pin)
{
  SDA_L(sda_port,sda_pin);
  SCL_H(scl_port,scl_pin);
  I2C_delay();


}
void I2C_WaitAck(GPIO_TypeDef* sda_port ,GPIO_TypeDef* scl_port ,uint16_t sda_pin ,uint16_t scl_pin)          //???:=1?ACK,=0?ACK
{
        SCL_L(scl_port,scl_pin);
        I2C_delay();
        SDA_H(sda_port,sda_pin) ;                       
        I2C_delay();
        SCL_H(scl_port,scl_pin) ;
        I2C_delay();
        if(SDA_read(sda_port,sda_pin))
        {
      SCL_L(scl_port,scl_pin) ;
 //     return FALSE;
        }
        SCL_L(scl_port,scl_pin) ;
//        return TRUE;
}

void I2C_SendByte(uint8_t SendByte,GPIO_TypeDef* sda_port ,GPIO_TypeDef* scl_port ,uint16_t sda_pin ,uint16_t scl_pin) //????????//
{
    uint8_t i=8;
    SCL_L(scl_port,scl_pin) ;
    while(i--)
    {        
      if(SendByte&0x80)
        SDA_H(sda_port,sda_pin) ;  
      else
        SDA_L(sda_port,sda_pin) ;   
        SendByte<<=1;
        I2C_delay();
        SCL_H(scl_port,scl_pin) ;
        I2C_delay();
        SCL_L(scl_port,scl_pin) ;
        I2C_delay();
    }
    SDA_H(sda_port,sda_pin) ;  
    I2C_delay();
}

uint8_t I2C_ReceiveByte(GPIO_TypeDef* sda_port ,GPIO_TypeDef* scl_port ,uint16_t sda_pin ,uint16_t scl_pin)  //????????//
{
    uint8_t i=8;
    uint8_t ReceiveByte=0;
    SCL_L(scl_port,scl_pin) ;
    I2C_delay();
    SDA_H(sda_port,sda_pin) ;
    I2C_delay();                               
    while(i--)
    {
      SCL_H(scl_port,scl_pin) ;
      I2C_delay();       
      ReceiveByte<<=1;      
      if(SDA_read(sda_port,sda_pin))
      {
        ReceiveByte|=0x01;
      }
      SCL_L(scl_port,scl_pin) ;
      I2C_delay();
    }

    return ReceiveByte;
}
void write_byte(unsigned char address,unsigned char reg,unsigned char command,GPIO_TypeDef* sda_port ,GPIO_TypeDef* scl_port ,uint16_t sda_pin ,uint16_t scl_pin)  //address+register+command
{

	I2C_Start(sda_port,scl_port,sda_pin,scl_pin);
	I2C_SendByte(address,sda_port,scl_port,sda_pin,scl_pin);
	I2C_Ack(sda_port,scl_port,sda_pin,scl_pin);
	I2C_SendByte(reg,sda_port,scl_port,sda_pin,scl_pin);
	I2C_Ack(sda_port,scl_port,sda_pin,scl_pin);
	I2C_SendByte(command,sda_port,scl_port,sda_pin,scl_pin);
	I2C_Ack(sda_port,scl_port,sda_pin,scl_pin);
	I2C_Stop(sda_port,scl_port,sda_pin,scl_pin);
}

unsigned char read_byte(unsigned char address,unsigned char reg,GPIO_TypeDef* sda_port ,GPIO_TypeDef* scl_port ,uint16_t sda_pin ,uint16_t scl_pin)         //address+register
{
	unsigned char dat;

	I2C_Start(sda_port,scl_port,sda_pin,scl_pin);
	I2C_SendByte(address,sda_port,scl_port,sda_pin,scl_pin);
	I2C_Ack(sda_port,scl_port,sda_pin,scl_pin);
	I2C_SendByte(reg,sda_port,scl_port,sda_pin,scl_pin);
	I2C_Ack(sda_port,scl_port,sda_pin,scl_pin);
	I2C_Start(sda_port,scl_port,sda_pin,scl_pin);
	I2C_SendByte(address+1,sda_port,scl_port,sda_pin,scl_pin);
	I2C_Ack(sda_port,scl_port,sda_pin,scl_pin);
	for(int i=0;i<5;i++)
        I2C_delay();
	dat=I2C_ReceiveByte(sda_port,scl_port,sda_pin,scl_pin);
	I2C_NoAck(sda_port,scl_port,sda_pin,scl_pin);
	I2C_Stop(sda_port,scl_port,sda_pin,scl_pin);
	return dat;
}

