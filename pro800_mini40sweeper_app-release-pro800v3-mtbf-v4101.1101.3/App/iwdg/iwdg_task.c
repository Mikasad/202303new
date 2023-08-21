
#include "iwdg_task.h"
#include "log.h"



/*
*********************************************************************************************************
*
*	模块名称 : 看门狗
*	文件名称 : iwdg_task.c
*	版    本 : 	V1.0.0
*	说    明 : 
*			1，看门狗功能
*			2，看门狗的软硬件初始化工作都在看门狗任务启动后进行
*			3，看门狗时间：3.2s
*	修改记录 :
*		版本号  		日期        作者     说明
*		V1.0.0    2020-03-31  杨鹏程   	正式发布
*	外设资源占用：
*		IWDG：占用
*	待优化：（需要优化但是未优化之前请保留该提示）
*********************************************************************************************************
*/
const static uint16_t m_task_percent80[configMAX_PRIORITIES] = //待优化
{
    1000*2*4/5, //defaultTask
    50*2*4/5,   //zdDriverTask
    20*2*4/5,   //DisinfectTask
    200*2*4/5,  //gyrohi216Task
    50*2*4/5,  //ultrasonicTask
    1000*2*4/5, //pcTask
    100*2*4/5,  //xd510Task
    200*2*4/5,  //motorDriverTask
    5*2*4/5,   //rfidTask
    100*2*4/5,  //antiDropCollisionTask
    1000*2*4/5, //ctrMoveTask
    1000*2*4/5, //canTask
    200*2*4/5,  //powerTask
    200*2*4/5,  //ioTask
};
static uint16_t m_task_run_count[configMAX_PRIORITIES] = {0};
static uint8_t  m_task_run_error_count[configMAX_PRIORITIES] = {0};


void FeedDog( uint32_t bit )
{
#if USE_HW_IWDG
    if( bit < configMAX_PRIORITIES )
    {
        m_task_run_count[bit]++;
    }
#else
    return;
#endif
}



#if USE_HW_IWDG

/*
*********************************************************************************************************
*	函 数 名: IWDGTask
*	功能说明: 
*		1,等待所有任务发事件标志过来。
*		2,如果有任务缺少，则打印输出
*		3,观察error code情况并输出
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级:  
*********************************************************************************************************
*/
void IWDGTask(void const *pvParameters)
{
	int i = 0;
	const TickType_t c_frequency = pdMS_TO_TICKS(2000);
	TickType_t xLastWakeTime;
	uint8_t  taskNumbers = (uint32_t)pvParameters;//iwdg任务之前的所有任务总数
    LogTaskAbnormal_t log = {0};
    
	if( taskNumbers>configMAX_PRIORITIES )
        taskNumbers = configMAX_PRIORITIES;
    
    vTaskDelay( 30000);

	/* 获取当前的系统时间 */
    xLastWakeTime = xTaskGetTickCount();

    while(1)
    {
        for(i=0; i<taskNumbers; i++ )
        {
            if( m_task_run_count[i] < m_task_percent80[i] )
            {
                m_task_run_error_count[i]++;
                if( m_task_run_error_count[i] >= 3 )
                {
                    m_task_run_error_count[i] = 0;
                    log.head.byte.id = ID_TASK_ABNORMAL;
                    log.id = i;
                    log.percent = (float)m_task_run_count[i]/(m_task_percent80[i]*5/4);
                    log.utc = GetUtc8();
                    xQueueSend( g_log_queue, &log, 2);
                }
            }
            else
                m_task_run_error_count[i] = 0;
            
            m_task_run_count[i] = 0;
        }
        /* vTaskDelayUntil是绝对延迟，vTaskDelay是相对延迟。*/
		vTaskDelayUntil(&xLastWakeTime, c_frequency);	
    }
}


int InitSoftwareIwdg(void)
{
	return 0;
}


#endif























