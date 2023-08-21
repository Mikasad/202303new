#include "stm32f4xx.h"



//APP
#include "BLDCdriver.h"
#include "decoder_task.h"
#include "apptask.h"



/*
*********************************************************************************************************
*
*	模块名称 : 编码计数采集模块
*	文件名称 : decoder_task.c
*	版    本 : 	V1.0.0
*	说    明 : 
*			1，该模块不涉及具体的硬件，主要是对编码计数数据进行采集
*	修改记录 :
*		版本号  		日期        作者     说明
*		V1.0.0    2018-11-05  杨臣   	正式发布
*	外设资源占用：
*		无
*	待优化：（需要优化但是未优化之前请保留该提示）
*		1，对于使用485进行命令读取位置信息的模式，此处的任务结构有些不适用，需要再做优化
*********************************************************************************************************
*/










#define READ_DECODER_CYCLE 30//单位ms


TaskHandle_t taskHandleDecoderCnt = NULL;



/*
*********************************************************************************************************
*	函 数 名: DecoderCntTask
*	功能说明: 编码器计数	
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 最高 
*********************************************************************************************************
*/
void DecoderCntTask(void *pvParameters)
{
	uint8_t cnt;
	TickType_t lastWakeTime;
	const TickType_t frequency = pdMS_TO_TICKS(10);//xms进行一次编码器单元计数
	uint32_t bitPos = (uint32_t)pvParameters;
	/* 获取当前的系统时间 */
    lastWakeTime = xTaskGetTickCount();

    while(1)
    {
		/* 喂狗 */
		TaskFeedDog(bitPos);
		/* vTaskDelayUntil是绝对延迟，vTaskDelay是相对延迟。*/
        vTaskDelayUntil(&lastWakeTime, frequency);
		DecoderUnitCnt();//单元计数
		cnt++;
		if(cnt >= 3)//n次单元计数之后(30ms)进行一次相对计算
		{
			DecoderRelativeCnt();//相对计数
		}
    }
}











