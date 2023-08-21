/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "semphr.h"

#include "imu_task.h"
#include "pc_correspond.h"
#include "ctr_move_task.h"
#include "can_task.h"
#include "normal_io.h"
#include "tcp_server.h"
#include "ultrasonic.h"
#include "motor_driver.h"
#include "power_ctr.h"
#include "scada.h"
#include "bms.h"
#include "XD510.h"
#include "anti_drop_collision.h"
#include "zd_driver.h"
#include "rfid_etag.h"
#include "flash_op.h"
#include "disinfect_bag.h"
#include "h25a15/app_h25a15.h"
#include "app_hub_motor.h"
#include "drv_backpack.h"
#include "hc_gpio.h"
#include "app_ethernet.h"
#include "tcp_server.h"
#include "app_bms.h"
#include "drv_h4aa30_usb_iap.h"
#include "app_ota.h"
#include "hc_gpio.h"
TaskHandle_t bms_tx_task_handle;
TaskHandle_t bms_rx_task_handle;
TaskHandle_t remote_module_task_handle;
TaskHandle_t anti_drop_task_handle;
extern __USB_ALIGN_BEGIN usb_core_instance usb_app_instance;
extern __USB_ALIGN_BEGIN USBH_HOST usb_app_host;

uint32_t g_thread_call_count[configMAX_PRIORITIES] = {0};

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize);

static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

/**
 * @brief 打印所有任务信息
 *
 */
void task_show_info(void)
{
    char *task_info = NULL;
    task_info = pvPortMalloc(1024);
    vTaskList((char *)task_info);

    printf("Name          \tStat\tPrio\tFStack\tnumber");
    printf("\r\n%s\r\n", task_info);
    vPortFree(task_info);
}

/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
void StartDefaultTask(void const *argument)
{
    TickType_t last_led_toggle_tick = 0;

    /* init code for LWIP */
    ETH_TaskInit();

    usb_host_init(&usb_app_instance, &usb_app_host, &USBH_MSC_cb, &USR_cb);

    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    vTaskDelay(500);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

    for (;;)
    {
        /* 板载运行灯控制 */
        if (xTaskGetTickCount() - last_led_toggle_tick >= 100)
        {
            last_led_toggle_tick = xTaskGetTickCount();
            HAL_GPIO_TogglePin(SYS_LED_GPIO_Port, SYS_LED_Pin);
        }
        ScadaTask();
        usb_host_mainprocess(&usb_app_instance, &usb_app_host); /*usb任务*/
        UsbBeep();                                              /*U盘升级驱动器蜂鸣器提升*/

        vTaskDelay(2);
    }
}


/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
    uint8_t priority = 1;

    /* 
        1.创建与上位机通信的网络任务;
        2.usb通信的后台任务;
        3.板载led闪烁任务;
        4.蜂鸣器提示任务;
    */
    xTaskCreate((TaskFunction_t)StartDefaultTask, "start_task", 2048, NULL, priority, NULL);
    priority++;
    /* 中大电机控制任务 */
    xTaskCreate((TaskFunction_t)ZdDriverTask, "zd_task", 256, NULL, priority, NULL);
    priority++;
    /* 消杀背包的开关任务??? */
    xTaskCreate((TaskFunction_t)DisinfectTask, "disinfect", 256, NULL, priority, NULL);
    priority++;
    /* imu获取任务 */
    xTaskCreate((TaskFunction_t)app_imu_task, "imu_rx", 512, NULL, priority, NULL);
    /* imu发送任务 */
    priority++;
    xTaskCreate((TaskFunction_t)app_imu_logic_task, "imu_tx", 256, NULL, priority, NULL);
    priority++;

    /* 超声波 */
    xTaskCreate((TaskFunction_t)UltrasonicTask, "ultrasonic", 256, NULL, priority, NULL);
    priority++;
    /* 上下位机通信(检测一些连接状态) */
    xTaskCreate((TaskFunction_t)PcCorrespondTask, "pc_com", 512, NULL, priority, NULL);
    priority++;
    /* xd510驱动器控制任务 */
    xTaskCreate((TaskFunction_t)Xd510Task, "xd510", 512, NULL, priority, NULL);
    priority++;
    /* 大能轮毂驱动器 */
    xTaskCreate((TaskFunction_t)MotorDriverTask, "motor_task", 256, NULL, priority, NULL);
    priority++;
    /* rfid任务 */
    xTaskCreate((TaskFunction_t)RfidTask, "rfid_task", 256, NULL, priority, NULL);
    priority++;
    /* can通信的防跌落传感器获取任务 */
    
    xTaskCreate((TaskFunction_t)AntiDropCollisionTask, "anti_logic", 256, NULL, priority, &anti_drop_task_handle);
    priority++;
    xTaskCreate((TaskFunction_t)app_anti_rx_process_Task, "anti_rx", 256, NULL, priority, NULL);
    priority++;

    /* 控制设备移动的逻辑 */
    xTaskCreate((TaskFunction_t)CtrMoveTask, "ctr_task", 256, NULL, priority, NULL);
    priority++;
    /* 两个can发送数据任务 */
    xTaskCreate((TaskFunction_t)CanTask, "can_task", 256, NULL, priority, NULL);
    priority++;

    xTaskCreate((TaskFunction_t)PowerCtrTask, "power_ctr", 256, NULL, priority, NULL);
    priority++;

    xTaskCreate((TaskFunction_t)NormalIoTask, "io_task", 256, NULL, priority, NULL);
    priority++;

    H25A_Motor_Task_Create();

    BackpackTaskInit();

    Hub_Motor_Task_Create();

    DevicePowerOn();
    
    xTaskCreate((TaskFunction_t)app_bms_tx_process_Task, "bms_tx", 512, NULL, 5, &bms_tx_task_handle);

    xTaskCreate((TaskFunction_t)app_bms_rx_process_Task, "bms_rx", 512, NULL, 6, &bms_rx_task_handle);

    USB_IAP_Task_Init();

    xTaskCreate(app_ota_msg_check, "ota_msg_check", 256, NULL, 5, NULL);  /* ota升级任务 */
    xTaskCreate(app_ota_task, "otatask", 256, NULL, 5, NULL);
    task_show_info();
}
