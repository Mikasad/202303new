/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "hc32_ll.h"

extern uint32_t g_thread_call_count[configMAX_PRIORITIES];

#define HAL_GetTick  SysTick_GetTick

void McuPowerDown(void);
void McuDrivePowerDown(void);
extern TaskHandle_t bms_tx_task_handle;
extern TaskHandle_t bms_rx_task_handle;
extern TaskHandle_t remote_module_task_handle;
extern TaskHandle_t anti_drop_task_handle;
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
