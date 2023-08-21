
#ifndef __SIM_I2C_H__
#define __SIM_I2C_H__
#include "stm32f4xx_hal.h"
void i2c_io_config(void);
void I2C_Start(GPIO_TypeDef* sda_port ,GPIO_TypeDef* scl_port ,uint16_t sda_pin ,uint16_t scl_pin);
void I2C_Stop(GPIO_TypeDef* sda_port ,GPIO_TypeDef* scl_port ,uint16_t sda_pin ,uint16_t scl_pin);
void I2C_Ack(GPIO_TypeDef* sda_port ,GPIO_TypeDef* scl_port ,uint16_t sda_pin ,uint16_t scl_pin);
void I2C_NoAck(GPIO_TypeDef* sda_port ,GPIO_TypeDef* scl_port ,uint16_t sda_pin ,uint16_t scl_pin);
void I2C_MasterAck(GPIO_TypeDef* sda_port ,GPIO_TypeDef* scl_port ,uint16_t sda_pin ,uint16_t scl_pin);
void I2C_WaitAck(GPIO_TypeDef* sda_port ,GPIO_TypeDef* scl_port ,uint16_t sda_pin ,uint16_t scl_pin);

void I2C_SendByte(uint8_t SendByte,GPIO_TypeDef* sda_port ,GPIO_TypeDef* scl_port ,uint16_t sda_pin ,uint16_t scl_pin);
uint8_t I2C_ReceiveByte(GPIO_TypeDef* sda_port ,GPIO_TypeDef* scl_port ,uint16_t sda_pin ,uint16_t scl_pin);
void write_byte(unsigned char address,unsigned char reg,unsigned char command,GPIO_TypeDef* sda_port ,GPIO_TypeDef* scl_port ,uint16_t sda_pin ,uint16_t scl_pin);  //address+register+command
unsigned char read_byte(unsigned char address,unsigned char reg,GPIO_TypeDef* sda_port ,GPIO_TypeDef* scl_port ,uint16_t sda_pin ,uint16_t scl_pin);         //address+register

#endif


/*******************************************
                SCL     SDA    
ultrasonic0     PA0     PA1
ultrasonic1     PA2     PA3
ultrasonic2     PA4     PA5
ultrasonic3     PA6     PA7
ultrasonic4     PB0     PB1
ultrasonic5     PB10    PB11
ultrasonic6     PB12    PB13
ultrasonic7     PB14    PB15
ultrasonic8     PA8     PA9
ultrasonic9     PA10    PA11
ultrasonic10    PA12    PA15
ultrasonic11    PB4     PB5
ultrasonic12    PB6     PB7
ultrasonic13    PC13    PC14
*******************************************/
