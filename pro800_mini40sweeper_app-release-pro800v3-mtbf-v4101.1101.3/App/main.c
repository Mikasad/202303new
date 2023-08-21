/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sysclock.h"
#include "hc_gpio.h"
#include "hc_can.h"
#include "hc_usart.h"
#include "hc_uart1.h"
#include "hc_uart2.h"
#include "hc_spi.h"
#include "hc_adc.h"
#include "hc_crc.h"
#include "hc_extint.h"
#include <stdio.h>
#include "flash_op.h"

uint8_t g_scheduler_start = 0;

/**
 * @brief 入口函数
 * @return int
 */
int main(void)
{
    SCB->VTOR = 0X00040000;

    LL_PERIPH_WE(LL_PERIPH_ALL);
    __disable_irq();
    
    BSP_CLK_Init(); /* 系统时钟初始化*/
    hc_gpio_init();
#if 0
	DDL_PrintfInit(CM_USART7, 115200UL, BSP_PRINTF_Preinit); /* 串口printf输出初始化*/
#else
    DDL_PrintfInit(CM_USART9, 460800UL, BSP_PRINTF_Preinit); /* 串口printf输出初始化*/
#endif
    HC_ExtiInit();

    /* 程序运行前检查钥匙开关是否关闭 */
    if (HAL_GPIO_ReadPin(MCU_KEY_GPIO_Port, MCU_KEY_Pin) == GPIO_PIN_RESET)
    {
        HAL_Delay(10); // 做短暂延时，防止误检测
        if (HAL_GPIO_ReadPin(MCU_KEY_GPIO_Port, MCU_KEY_Pin) == GPIO_PIN_RESET)
        {
            McuPowerDown(); // 钥匙关闭，设备重启
        }
    }
    /* POWER UP */
    /* 启动另外一路24V供电电源，钥匙关闭后设备不会立即掉电 */
    HAL_GPIO_WritePin(POWER_UP_GPIO_Port, POWER_UP_Pin, GPIO_PIN_SET);

    /* enable all power */
    /* 依次开启电源（其实这时已经全部打开了）*/
    HAL_GPIO_WritePin(V12_EN_GPIO_Port, V12_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(DC5V_EN_1_GPIO_Port, DC5V_EN_1_Pin, GPIO_PIN_SET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(DC3V3_EN_GPIO_Port, DC3V3_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(V3V3_EN_1_GPIO_Port, V3V3_EN_1_Pin, GPIO_PIN_SET);

    hc_crc32_init();
    bsp_usart1_init();
    bsp_usart2_init();
    hc_usart3_init(57600);
    hc_usart4_init();
    hc_usart5_init();
    hc_usart6_init();

    hc_spi_init();
    can1_config_init();
    can2_config_init();

    hc_dma_init();
    hc_adc1_init();
    hc_adc3_init();

    ADC_Start(CM_ADC1);
    ADC_Start(CM_ADC3);

    /* 检查ota参数存储区，根据存储区内的参数更新备份区 */
    InitSoftwareFlashOp();

    /* 初始化各任务 */
    extern void MX_FREERTOS_Init(void);
    MX_FREERTOS_Init();

    /* 设置系统启动的标志用于其他地方判断系统是否起来（os应该提供了类似的函数）*/
    g_scheduler_start = 1;

    vTaskStartScheduler();

    while (1)
    {
        ;
    }
}

/*
 *关闭所有可控电源，复位mcu
 */
void McuPowerDown(void)
{

    HAL_GPIO_WritePin(DC3V3_EN_GPIO_Port, DC3V3_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(V3V3_EN_1_GPIO_Port, V3V3_EN_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DC5V_EN_1_GPIO_Port, DC5V_EN_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(V12_EN_GPIO_Port, V12_EN_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(POWER_UP_GPIO_Port, POWER_UP_Pin, GPIO_PIN_RESET);
    NVIC_SystemReset(); // 复位
}

/*
 *关闭所有可控电源
 */
void McuDrivePowerDown(void)
{
    HAL_GPIO_WritePin(DC3V3_EN_GPIO_Port, DC3V3_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(V3V3_EN_1_GPIO_Port, V3V3_EN_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DC5V_EN_1_GPIO_Port, DC5V_EN_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(V12_EN_GPIO_Port, V12_EN_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
}

