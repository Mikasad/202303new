/*
*******************************************************************************
 *版    权： 2021-xxxx,  GaussianRobot
 *文 件 名： stm32f4xx.mc.it.c中断函数
 *简    介： 用于处理所有中断函数
 *作    者： LMmotor\忘川
 *日    期： 2022.12.6
 *功能描述：
*******************************************************************************
 *备注：
*******************************************************************************
*/
#include "mc_type.h"
#include "mc_tasks.h"
#include "ui_task.h"
#include "parameters_conversion.h"
#include "motorcontrol.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "canopen_od.h"
#include "canopen_pdo.h"
#include "ds402.h"
#include "bsp_app_function.h"
#include "bsp_usart1.h"
#include "Usart_Labview.h"


s16 HALL_CH1_Value1=0,HALL_CH1_Value2=0,HALL_CH1_ValueDelta=0,HALL_CH1_ValueDelta1,HALL_CH1_ValueDelta2,HALL_CH1_ValueOffset=0;     /*PWM捕获初始定位参数，未使用但保留*/
s16 HALL_CH2_Value1=0,HALL_CH2_Value2=0,HALL_CH2_ValueDelta=0,HALL_CH2_ValueDelta1,HALL_CH2_ValueDelta2,HALL_CH2_ValueOffset=0;
s16 PWM_Init_Electrical_Angle=0,PWM_Init_Electrical_Angle2=0;                                                                       /*同上，PWM定位得到的初始角度*/

uint8_t debug_motor1_exit= 0;                                 /*用于测试失速问题*/
uint8_t debug_motor2_exit= 0;                                 /*用于测试失速问题*/
int16_t debug_motor1_Angle_Compensation = 0;                  /*用于测试失速问题*/
int16_t debug_motor2_Angle_Compensation = 0;                  /*用于测试失速问题*/
#define   BrakeFilterDeep  5                                  /*brake中断滤波次数*/
uint16_t BrakeFiltersCnt[2] = {0};                            /*brake中断滤波计次*/

extern IWDG_HandleTypeDef hiwdg;
extern STM_Handle_t STM[NBR_OF_MOTORS];
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef husart_debug;
extern PWMC_Handle_t * pwmcHandle[NBR_OF_MOTORS];

u16 EMG_State,EMG_State2;                                     /*急停*/
u32 led_1ms_cnt = 0;                                          /*LED指示灯计数*/

u8 datalens = 0;                                              /*串口接收的数据长度*/
extern u8 g_uart_recv_ok;                                     /*串口接收完成标志*/
#define Labview_uart_Flag1CNT   2                             /*用于Labview_uart打印设置周期*/
uint8_t Labview_uart_Flag1 = 0;                               /*用于Labview_uart打印循环周期计数*/
uint16_t CNT_1MS[MAX_AXES];                                   /*1ms计数*/

void ADC_IRQHandler(void);
void TIMx_UP_M1_IRQHandler(void);
void TIMx_BRK_M1_IRQHandler(void);
void SPD_TIM_M1_IRQHandler(void);
void TIMx_UP_M2_IRQHandler(void);
void TIMx_BRK_M2_IRQHandler(void);
void SPD_TIM_M2_IRQHandler(void);
void HardFault_Handler(void);
void SysTick_Handler(void);
void EXTI3_IRQHandler (void);
void EXTI9_5_IRQHandler(void);

uint8_t PrevHallState1,PrevHallState2,HallState1,HallState2;                   /*根据霍尔状态的变换确定初始角度给定*/
int16_t HAL_Init_Electrical_Angle2 = 0,HAL_CommutationAngle2 = 0;
int16_t HAL_Init_Electrical_Angle = 0,HAL_CommutationAngle = 0;

u16 Motor1HallGetTimCNT[200],Motor2HallGetTimCNT[200];                         /*为了查看定时器CNT值，测试*/
s16 Hall_ElAngle1[7],Hall_ElAngle2[7];                                         /*霍尔学习时使用记录每个霍尔状态的角度值*/

extern s16 ENC_ElAngle,ENC_ElAngle2;
extern int32_t hSpeedRef_Pos;
/*------------------------------------------------
* @function :ADC中断函数,注入通道的电流采样
* @input    :
* @output   :
* @explain  :tim1(100us)和tim8(100us)的ch4轮流触发，周期50us
* @author   :wangchuan
* @date     :2022/12/22
------------------------------------------------*/
void ADC_IRQHandler(void)
{
//   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);                        /*用于测试ADC执行周期，保留*/
    if(LL_ADC_IsActiveFlag_JEOS(ADC1))
    {
        ADC1->SR &= ~(uint32_t)(LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JSTRT);          /*清除中断标志位*/
        /*高频任务，获取电角度并且进行FOC运算*/
        TSK_HighFrequencyTask();                                                /*GUI, this section is present only if DAC is disabled*/
    }
#ifdef ADC3
    else
    {
        ADC3->SR &= ~(uint32_t)(LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JSTRT);          /*清除中断标志位*/
        /*高频任务，获取电角度并且进行FOC运算*/
        TSK_HighFrequencyTask();                                                /*GUI, this section is present only if DAC is disabled*/
    }
#endif
    CanLoadRate.timer++;
    if(RPDO_SYNC_SIGN & RPDO_SYNC_READY)
    {
        RPDO_SYNC_SIGN &= ~RPDO_SYNC_READY;
        _RPOD_SyncEvent(&CANopen_Drive,pRpdoDirectPar,1);
    }

//  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);                       /*用于测试ADC执行周期，保留*/
}

/*------------------------------------------------
* @function :TIM1更新中断
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2022/12/22
------------------------------------------------*/
void TIMx_UP_M1_IRQHandler(void)
{
    LL_TIM_ClearFlag_UPDATE(PWM_Handle_M1.pParams_str->TIMx);
    R3_2_TIMx_UP_IRQHandler(&PWM_Handle_M1);
    TSK_DualDriveFIFOUpdate( M1 );
}
/*------------------------------------------------
* @function :TIM8更新中断
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2022/12/22
------------------------------------------------*/
void TIMx_UP_M2_IRQHandler(void)
{
    LL_TIM_ClearFlag_UPDATE(PWM_Handle_M2.pParams_str->TIMx);
    R3_2_TIMx_UP_IRQHandler(&PWM_Handle_M2);
    TSK_DualDriveFIFOUpdate( M2 );
}

/*------------------------------------------------
* @function :M1刹车中断
* @input    :
* @output   :
* @explain  :定时器中配置的PWM自动输出使能功能，brake受到干扰可以滤除
             但是不影响硬件寿命，因为在进入中断之前硬件已经关闭PWM输出
             下一个周期继续输出， BrakeFilterDeep 滤波5次
             BrakeFiltersCnt 每100ms清除一次
* @author   :wangchuan
* @date     :2022/12/22
------------------------------------------------*/
void TIMx_BRK_M1_IRQHandler(void)
{
    BrakeFiltersCnt[M1]++;
    if(BrakeFiltersCnt[M1] > BrakeFilterDeep)
    {
        R3_2_SwitchOffPWM( pwmcHandle[M1] );                                 /*关闭自动输出使能并且关闭所有PWM输出*/
        if (LL_TIM_IsActiveFlag_BRK(PWM_Handle_M1.pParams_str->TIMx))
        {
            LL_TIM_ClearFlag_BRK(PWM_Handle_M1.pParams_str->TIMx);
            R3_2_BRK_IRQHandler(&PWM_Handle_M1);                             /*报brake错误*/
        }
        /* Systick is not executed due low priority so is necessary to call MC_Scheduler here.*/
//        MC_Scheduler();//其中声明的变量static volatile uint16_t hMFTaskCounterM1 = 0; 类型不适合进行两个文件的调用，存在严重问题
    }
    else
    {
        LL_TIM_ClearFlag_BRK(PWM_Handle_M1.pParams_str->TIMx);
    }
}
/*------------------------------------------------
* @function :M2刹车中断
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2022/12/22
------------------------------------------------*/
void TIMx_BRK_M2_IRQHandler(void)
{
    BrakeFiltersCnt[M2]++;
    if(BrakeFiltersCnt[M2] > BrakeFilterDeep)
    {
        R3_2_SwitchOffPWM( pwmcHandle[M2] );                                /*关闭自动输出使能并且关闭所有PWM输出*/
        if (LL_TIM_IsActiveFlag_BRK(PWM_Handle_M2.pParams_str->TIMx))
        {
            LL_TIM_ClearFlag_BRK(PWM_Handle_M2.pParams_str->TIMx);
            R3_2_BRK_IRQHandler(&PWM_Handle_M2);                            /*报brake错误*/
        }
        /* Systick is not executed due low priority so is necessary to call MC_Scheduler here.*/
//        MC_Scheduler();
    }
    else
    {
        LL_TIM_ClearFlag_BRK(PWM_Handle_M2.pParams_str->TIMx);
    }
}

/*------------------------------------------------
* @function :霍尔中断
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2022/12/22
------------------------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if((GPIO_Pin == M1_HALL_U_Pin )||(GPIO_Pin == M1_HALL_V_Pin )||(GPIO_Pin == M1_HALL_W_Pin ))     /*M1电机霍尔中断*/
    {
        HALL_CC_First++;                                                                             /*进入霍尔中断的次数*/
        PrevHallState1 = HallState1;
        HallState1 = HALL_GetPhase1();                                                               /*获取当前霍尔状态*/
        if(HallStudyFlag1 == 1)                                                                      /*霍尔学习*/
        {
            if(HallStudyCNT<Hall_EAC_Ticks)                                                          /*3秒*/
            {
                Angle_Switch = 0;
            }
            else
            {
                Hall_ElAngle1[HallState1] = ENC_ElAngle;                                             /*获取电角度*/
                if(HallState1 == 5)                                                                  /*霍尔角度转过一周，相当于1对极*/
                {
                    Hall_AvElAngle1CNT++;
                    Hall_AvElAngle1Sum += Hall_ElAngle1[5];                                          /*电角度累加和*/
                }
            }
        }
        else
        {

            HALL_CC_First11++;                                                                        /*测试*/
            Hall_AvElAngle1CNT =0;                                                                    /*电角度周期*/
            Hall_AvElAngle1Sum =0;                                                                    /*霍尔学习电角度和清零*/
            Motor1HallGetTimCNT[HALL_CC_First11] = LL_TIM_GetCounter ( TIM4 );                        /*为了查看定时器CNT值，测试*/
            if(HALL_CC_First11>=99)                                                                   /*测试*/
            {
                HALL_CC_First11=0;
            }

            if(HALL_CC_First<2)                                                                        /*首次启动电机校准电机位置*/
            {
                switch ( HallState1 )                                                                  /*根据霍尔状态大概判断电机启动位置*/
                {
                case STATE_5:
                    if ( PrevHallState1 == STATE_4 )                                                   /*霍尔状态由4->5*/
                    {
//						HAL_CommutationAngle1 = (int16_t)(HALL_PHASE_SHIFT1 * 65536/360);//
                        HAL_CommutationAngle = (int16_t)(HALL_PHASE_SHIFT);
                    }
                    else if ( PrevHallState1 == STATE_1 )                                              /*霍尔状态由1->5*/
                    {
                        HAL_CommutationAngle = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT) + S16_60_PHASE_SHIFT );
                    }
                    else
                    {
                        if(HALL_CC_First > 0)             /*HALL_CC_First 是uint8_t 类型，如果为0再--，那就会直接变成255*/
                        {
                            HALL_CC_First--;                                                               /*如果进入此处说明本次定位错误，需要重新定位*/
                        }
                        debug_motor1_exit = 5;                                                         /*用于测试失速问题*/
                    }
                    break;
                case STATE_1:
                    if ( PrevHallState1 == STATE_5 )
                    {
                        HAL_CommutationAngle = (int16_t)(HALL_PHASE_SHIFT) + S16_60_PHASE_SHIFT;
                    }
                    else if ( PrevHallState1 == STATE_3 )
                    {
                        HAL_CommutationAngle = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT) + S16_120_PHASE_SHIFT );
                    }
                    else
                    {
                        if(HALL_CC_First > 0)             /*HALL_CC_First 是uint8_t 类型，如果为0再--，那就会直接变成255*/
                        {
                            HALL_CC_First--;                 /*如果进入此处说明本次定位错误，需要重新定位*/
                        }
                        debug_motor1_exit = 1;//用于测试失速问题
                    }
                    break;

                case STATE_3:
                    if ( PrevHallState1 == STATE_1 )
                    {

                        HAL_CommutationAngle = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT) + S16_120_PHASE_SHIFT );
                    }
                    else if ( PrevHallState1 == STATE_2 )
                    {

                        HAL_CommutationAngle = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT) + S16_120_PHASE_SHIFT +
                                                            S16_60_PHASE_SHIFT );
                    }
                    else
                    {
                        if(HALL_CC_First > 0)                                      /*HALL_CC_First 是uint8_t 类型，如果为0再--，那就会直接变成255*/
                        {
                            HALL_CC_First--;                                       /*如果进入此处说明本次定位错误，需要重新定位*/
                        }
                        debug_motor1_exit = 3;
                    }

                    break;
                case STATE_2:
                    if ( PrevHallState1 == STATE_3 )
                    {

                        HAL_CommutationAngle = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT) + S16_120_PHASE_SHIFT
                                                            + S16_60_PHASE_SHIFT );
                    }
                    else if ( PrevHallState1 == STATE_6 )
                    {

                        HAL_CommutationAngle = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT) - S16_120_PHASE_SHIFT );
                    }
                    else
                    {
                        if(HALL_CC_First > 0)                                        /*HALL_CC_First 是uint8_t 类型，如果为0再--，那就会直接变成255*/
                        {
                            HALL_CC_First--;                                         /*如果进入此处说明本次定位错误，需要重新定位*/
                        }
                        debug_motor1_exit = 2;//用于测试失速问题
                    }
                    break;
                case STATE_6:
                    if ( PrevHallState1 == STATE_2 )
                    {

                        HAL_CommutationAngle = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT) - S16_120_PHASE_SHIFT );         /*-20600 - 21845 = -42445*/
                    }
                    else if ( PrevHallState1 == STATE_4 )
                    {

                        HAL_CommutationAngle = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT) - S16_60_PHASE_SHIFT );
                    }
                    else
                    {
                        if(HALL_CC_First > 0)                                         /*HALL_CC_First 是uint8_t 类型，如果为0再--，那就会直接变成255*/
                        {
                            HALL_CC_First--;                                          /*如果进入此处说明本次定位错误，需要重新定位*/
                        }
                        debug_motor1_exit = 6;//用于测试失速问题
                    }
                    break;

                case STATE_4:
                    if ( PrevHallState1 == STATE_6 )
                    {

                        HAL_CommutationAngle = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT) - S16_60_PHASE_SHIFT );          /*-20600 - 10922 = -31522*/
                    }
                    else if ( PrevHallState1 == STATE_5 )
                    {

                        HAL_CommutationAngle = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT) );                               /*-20600*/
                    }
                    else
                    {
                        if(HALL_CC_First > 0)                                            /*HALL_CC_First 是uint8_t 类型，如果为0再--，那就会直接变成255*/
                        {
                            HALL_CC_First--;                                             /*如果进入此处说明本次定位错误，需要重新定位*/
                        }
                        debug_motor1_exit = 4;//用于测试失速问题
                    }
                    break;
                default:
                    if(HALL_CC_First > 0)                                                /*HALL_CC_First 是uint8_t 类型，如果为0再--，那就会直接变成255*/
                    {
                        HALL_CC_First--;                                                 /*如果进入此处说明本次定位错误，需要重新定位*/
                    }
                    break;
                }

                if(HallStudyFlag1 == 0)                                                                                   /*如果不进行霍尔学习，则计算换向角*/
                {
                    if(HALL_CC_First == 1 && HAL_CommutationAngle != 0)                                                   /*新增 HAL_CommutationAngle ！= 0 没有排除正常定位情况下HAL_CommutationAngle = 0的情况，避免机器跑飞*/
                    {
                        ENCODER_M1.Angle_Compensation   = HAL_CommutationAngle - ENCODER_M1._Super.hElAngle ;
                        debug_motor1_Angle_Compensation = HAL_CommutationAngle - ENCODER_M1._Super.hElAngle ;             /*用于查看变量值*/
                    }
                }
                else                                                                                                      /*如果进行霍尔学习则换向角清零*/
                {
                    ENCODER_M1.Angle_Compensation = 0 ;
                }
            }
            else
            {
                HALL_CC_First = 2;
            }
        }
    }

    if((GPIO_Pin == M2_HALL_U_Pin )||(GPIO_Pin == M2_HALL_V_Pin )||(GPIO_Pin == M2_HALL_W_Pin ))              /*M2电机霍尔中断*/
    {
        HALL2_CC_First++;
        PrevHallState2 = HallState2;//用于判断方向
        HallState2 = HALL_GetPhase2();;

        if(HallStudyFlag2 == 1)//霍尔学习
        {
            hSpeedRef_Pos = 0;
            if(HallStudyCNT2<Hall_EAC_Ticks)
            {
                Angle_Switch2 = 0;

            }
            else
            {
                Hall_ElAngle2[HallState2] = ENC_ElAngle2;

                if(HallState2 == 5)
                {
                    Hall_AvElAngle2CNT++;
                    Hall_AvElAngle2Sum += Hall_ElAngle2[5];
                }
            }
        }
        else
        {
            Hall_AvElAngle2CNT =0;
            Hall_AvElAngle2Sum =0;
            if(HALL2_CC_First<2)
            {
                switch ( HallState2 )
                {
                case STATE_5:
                    if ( PrevHallState2 == STATE_4 )
                    {
                        HAL_CommutationAngle2 = (int16_t)(HALL_PHASE_SHIFT2);
                    }
                    else if ( PrevHallState2 == STATE_1 )
                    {
                        HAL_CommutationAngle2 = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT2) + S16_60_PHASE_SHIFT );
                    }
                    else
                    {
                        if(HALL2_CC_First > 0)
                        {
                            HALL2_CC_First--;
                        }
                        debug_motor2_exit = 5;
                    }
                    break;

                case STATE_1:
                    if ( PrevHallState2 == STATE_5 )
                    {
                        HAL_CommutationAngle2 = (int16_t)(HALL_PHASE_SHIFT2) + S16_60_PHASE_SHIFT;
                    }
                    else if ( PrevHallState2 == STATE_3 )
                    {
                        HAL_CommutationAngle2 = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT2) + S16_120_PHASE_SHIFT );
                    }
                    else
                    {
                        if(HALL2_CC_First > 0)
                        {
                            HALL2_CC_First--;
                        }
                        debug_motor2_exit = 1;
                    }
                    break;

                case STATE_3:
                    if ( PrevHallState2 == STATE_1 )
                    {
                        HAL_CommutationAngle2 = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT2) + S16_120_PHASE_SHIFT );
                    }
                    else if ( PrevHallState2 == STATE_2 )
                    {
                        HAL_CommutationAngle2 = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT2) + S16_120_PHASE_SHIFT +
                                                             S16_60_PHASE_SHIFT );
                    }
                    else
                    {
                        if(HALL2_CC_First > 0)
                        {
                            HALL2_CC_First--;
                        }
                        debug_motor2_exit = 3;
                    }

                    break;

                case STATE_2:
                    if ( PrevHallState2 == STATE_3 )
                    {

                        HAL_CommutationAngle2 = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT2) + S16_120_PHASE_SHIFT
                                                             + S16_60_PHASE_SHIFT );
                    }
                    else if ( PrevHallState2 == STATE_6 )
                    {

                        HAL_CommutationAngle2 = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT2) - S16_120_PHASE_SHIFT );
                    }
                    else
                    {
                        if(HALL2_CC_First > 0)
                        {
                            HALL2_CC_First--;
                        }
                        debug_motor2_exit = 2;
                    }
                    break;

                case STATE_6:
                    if ( PrevHallState2 == STATE_2 )
                    {
                        HAL_CommutationAngle2 = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT2) - S16_120_PHASE_SHIFT );
                    }
                    else if ( PrevHallState2 == STATE_4 )
                    {
                        HAL_CommutationAngle2 = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT2) - S16_60_PHASE_SHIFT );
                    }
                    else
                    {
                        if(HALL2_CC_First > 0)
                        {
                            HALL2_CC_First--;
                        }
                        debug_motor2_exit = 6;
                    }
                    break;

                case STATE_4:
                    if ( PrevHallState2 == STATE_6 )
                    {
                        HAL_CommutationAngle2 = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT2) - S16_60_PHASE_SHIFT );
                    }
                    else if ( PrevHallState2 == STATE_5 )
                    {
                        HAL_CommutationAngle2 = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT2) );
                    }
                    else
                    {
                        if(HALL2_CC_First > 0)
                        {
                            HALL2_CC_First--;
                        }
                        debug_motor2_exit = 4;
                    }
                    break;

                default:
                    if(HALL2_CC_First > 0)
                    {
                        HALL2_CC_First--;
                    }
                    break;
                }
								
                if(HallStudyFlag2 == 0)
                {
                    if(HALL2_CC_First == 1 && HAL_CommutationAngle2 != 0)
                    {
                        ENCODER_M2.Angle_Compensation = HAL_CommutationAngle2 - ENCODER_M2._Super.hElAngle ;
                        debug_motor2_Angle_Compensation = HAL_CommutationAngle2 - ENCODER_M2._Super.hElAngle ;
                    }
                }
                else
                {
                    ENCODER_M2.Angle_Compensation = 0 ;
                }
            }
            else
            {
                HALL2_CC_First = 2;
            }
        }
    }
}
/*------------------------------------------------
* @function :TIM4比较中断，M1编码器
* @input    :
* @output   :
* @explain  :用于编码器PWM模式捕获进行初始定位，S02A11-C3软件版本未使用中断已经关闭
* @author   :wangchuan
* @date     :2022/12/22
------------------------------------------------*/
void SPD_TIM_M1_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_CC1 (TIM4))                                /*TIM4 CH1通道比较中断*/
    {
        HallState1 = (GPIOD->IDR>>12)&(0x1);                           /*为什么要获取编码器A的电平状态*/
        if(HallState1 == 1)
        {
//			TIM4->CNT = 0;
            HALL_CH1_Value1 = LL_TIM_GetCounter ( TIM4 );              /*获取编码器计数值*/
//			TIM4->CCER |=0x0002 ;
        }
        else
        {
            HALL_CH1_Value2 = LL_TIM_GetCounter ( TIM4 );              /*获取编码器计数值*/
            if(HALL_CH1_Value2>HALL_CH1_Value1)                        /*编码器计数值没有溢出*/
            {
                HALL_CH1_ValueDelta = HALL_CH1_Value2 - HALL_CH1_Value1 ;
            }
            else                                                       /*如果编码器计数值溢出*/
            {
                HALL_CH1_ValueDelta = HALL_CH1_Value2 + 65536- HALL_CH1_Value1 ;
            }
            if(HALL_CH1_ValueDelta>111)                                                        /*？？？？？？？？？？*/
            {
                if(HALL_CH1_ValueDelta>29000)
                {
                    HALL_CH1_ValueDelta = 29000;
                }

                HALL_CH1_ValueDelta1 = 29000 - HALL_CH1_ValueDelta;
                HALL_CH1_ValueDelta1 = (HALL_CH1_ValueDelta1%2063) ;                           /*？？？？？？？？？？*/
                HALL_CH1_ValueDelta2 = (HALL_CH1_ValueDelta1 - HALL_CH1_ValueOffset)*31.76;    /*？？？？？？？？？？*/

                if(HALL_CC_First<1 )
                {
                    PWM_Init_Electrical_Angle =  HALL_CH1_ValueDelta2;                         /*重要*/
                    HALL_CC_First++;
                }
            }
            else
            {
                HALL_CH1_ValueDelta1 = 0;
            }
//			TIM4->CCER &=0xfffd ;
        }
        LL_TIM_ClearFlag_CC1(TIM4);                                                            /*清除TIM4CC1比较中断*/
    }
    else if (LL_TIM_IsActiveFlag_CC2 (TIM4))
    {
        LL_TIM_ClearFlag_CC2(TIM4);
        HallState1 = (GPIOD->IDR>>12)&(0x2);
        if(HallState1 == 2)
        {

            HALL_CH2_Value1 = LL_TIM_GetCounter ( TIM4 );
        }
        else
        {
            HALL_CH2_Value2 = LL_TIM_GetCounter ( TIM4 );
            if(HALL_CH2_Value2>HALL_CH2_Value1)
            {
                HALL_CH2_ValueDelta = HALL_CH2_Value2 - HALL_CH2_Value1 ;
            }
            else
            {
                HALL_CH2_ValueDelta = HALL_CH2_Value2 - HALL_CH2_Value1 + 65536;
            }

            if(HALL_CH2_ValueDelta>111)
            {
                if(HALL_CH2_ValueDelta>29000)
                {
                    HALL_CH2_ValueDelta = 29000;
                }

                HALL_CH2_ValueDelta1 = 29000 - HALL_CH2_ValueDelta;
                HALL_CH2_ValueDelta1 = (HALL_CH2_ValueDelta1%2063) ;
                HALL_CH2_ValueDelta2 = (HALL_CH2_ValueDelta1 - HALL_CH2_ValueOffset)*31.76;

                if(HALL2_CC_First<1 )
                {
                    PWM_Init_Electrical_Angle2 =  HALL_CH2_ValueDelta2;
                    HALL2_CC_First++;
                }
            }
            else
            {
                HALL_CH2_ValueDelta1 = 0;
            }

        }

    }
}
/*------------------------------------------------
* @function :TIM2更新中断
* @input    :
* @output   :
* @explain  :计数CNT溢出次数，暂未使用，中断已经关闭
* @author   :wangchuan
* @date     :2022/12/22
------------------------------------------------*/
void SPD_TIM_M2_IRQHandler(void)
{
    if (LL_TIM_IsEnabledIT_UPDATE (ENCODER_M2.TIMx) && LL_TIM_IsActiveFlag_UPDATE (ENCODER_M2.TIMx))
    {
        LL_TIM_ClearFlag_UPDATE(ENCODER_M2.TIMx);
        ENC_IRQHandler(&ENCODER_M2);
    }
    else
    {

    }
}
/*------------------------------------------------
* @function :串口接收中断
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2022/12/22
------------------------------------------------*/
void USART3_IRQHandler(void)
{
    if(__HAL_UART_GET_FLAG(&husart_debug,USART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&husart_debug);                          /*清除uart3空闲中断标志位*/
        HAL_UART_DMAStop(&husart_debug);                                   /*停止uart3的DMA传输*/
        datalens =50 - husart_debug.hdmarx->Instance->NDTR;                /*得到DMA传输数据的长度*/
        if(datalens < 50)                                                  /*数据长度没有超过50的*/
        {
            if(UsartRxBuffer[0] == 0xAA && (UsartRxBuffer[datalens-1] == 0x2F) && UsartRxBuffer[datalens-2] == datalens) /*判断帧头和帧尾*/
            {
                g_uart_recv_ok =1;                                         /*接收完成一包数据，在Usart_Labview_Analyze函数中开启处理*/
            }
            else
            {
            }
        }
        else
        {

        }
    }
    HAL_UART_Receive_DMA(&husart_debug,UsartRxBuffer,50);
    HAL_UART_IRQHandler(&husart_debug);
}
/*------------------------------------------------
* @function :硬件中断
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2022/12/22
------------------------------------------------*/
void HardFault_Handler(void)
{
    TSK_HardwareFaultTask();
    while (1)
    {

        if (LL_USART_IsActiveFlag_ORE(pUSART.USARTx)) /* Overrun error occurs */
        {
            /* Send Overrun message */
            UFCP_OVR_IRQ_Handler(&pUSART);
            LL_USART_ClearFlag_ORE(pUSART.USARTx); /* Clear overrun flag */
            UI_SerialCommunicationTimeOutStop();
        }

        if (LL_USART_IsActiveFlag_TXE(pUSART.USARTx))
        {
            UFCP_TX_IRQ_Handler(&pUSART);
        }

        if (LL_USART_IsActiveFlag_RXNE(pUSART.USARTx)) /* Valid data have been received */
        {
            uint16_t retVal;
            retVal = *(uint16_t*)(UFCP_RX_IRQ_Handler(&pUSART,LL_USART_ReceiveData8(pUSART.USARTx)));
            if (retVal == 1)
            {
                UI_SerialCommunicationTimeOutStart();
            }
            if (retVal == 2)
            {
                UI_SerialCommunicationTimeOutStop();
            }
        }
        else
        {
        }

    }
}
/*------------------------------------------------
* @function :滴答定时器中断
* @input    :请填写
* @output   :请填写
* @explain  :500us执行一次
* @author   :忘川
* @date     :2022/12/22
------------------------------------------------*/
void SysTick_Handler(void)
{
    static uint8_t SystickDividerCounter = SYSTICK_DIVIDER;
    if (SystickDividerCounter == SYSTICK_DIVIDER)  /*1ms周期*/
    {
        SystickDividerCounter = 0;                         /*2分频计数*/
        led_1ms_cnt++;                                     /*持续计时未清0，4294967295ms/1000=(s)/60=(min)/60=(h)/24=49(Day)*/
        Usart_Labview_Analyze();                           /*串口labview指令解析*/
        Damping_Of_Motor();                                /*阻尼模式*/

        if(led_1ms_cnt%10==0)                              /*10ms执行LED指示灯闪烁*/
        {
            Display_error();                               /*整合两个电机的错误到一个32位变量当中*/
            DisplayErrorLed_Handle();                      /*错误指示灯*/
        }

        Sci_Monitor();                                     /*USART3串口数据传输监控*/
        Sci_Stack_Monitor(&USART3_EMAIL.State_monitor);    /*USART3串口队列传输监控*/

        if(led_1ms_cnt%100 == 0)                           /*100ms清除一次brake滤波计数值,并且喂狗一次*/
        {
            HAL_IWDG_Refresh(&hiwdg);
            Clear_Brake_Filters_Cnt();
        }

        M1M2_Initial_Position();                           /*电机初始位置学习*/

        CNT_1MS[M1]++;
        CNT_1MS[M2]++;
        HallStudyCNT++;
        HallStudyCNT2++;
        Labview_uart_Flag ++;
        Labview_uart_Flag1++;
        HAL_IncTick();                                      /*滴答时钟1ms计数*/
        HAL_SYSTICK_IRQHandler();                           /*无内容考虑删除*/

    }
    else
    {

    }
    SystickDividerCounter ++;
    ProceedDriveStateChange();                              /*402状态机*/
    ProceedDriverHandler();                                 /*402状态机*/
    PendingOptionCode();                                    /*402状态机*/
    MC_RunMotorControlTasks();//500us
}
/*------------------------------------------------
* @function :外部急停中断
* @input    :请填写
* @output   :请填写
* @explain  :请填写
* @author   :忘川
* @date     :2022/12/22
------------------------------------------------*/
void EXTI0_IRQHandler (void)
{
    if ( LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_0) )
    {
        EMG_State = (Start_Stop_GPIO_Port->IDR>>0)&(0x01);        /*获取当前引脚状态*/
        if(EMG_State == STOPISON)
        {
            for(u16 i=0; i<10000; i++);                           /*相当于滤波*/
            EMG_State2 = (Start_Stop_GPIO_Port->IDR>>0)&(0x01);
            EMG_State =  (Start_Stop_GPIO_Port->IDR>>0)&(0x01);
            if(EMG_State == STOPISON)                                    /*急停被按下*/
            {
                pCtrlPar[M1].SetVelMotor = 0;
                pCtrlPar[M2].SetVelMotor = 0;
                pCtrlPar[M1].M1M2_VelSet = 0;
            }
        }
        LL_EXTI_ClearFlag_0_31 (LL_EXTI_LINE_0);                   /*清除中断标志*/
    }
}
/*------------------------------------------------
* @function :M1霍尔
* @input    :请填写
* @output   :请填写
* @explain  :请填写
* @author   :忘川
* @date     :2022/12/22
------------------------------------------------*/
void EXTI9_5_IRQHandler(void)
{
    if ( LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_5|LL_EXTI_LINE_6|LL_EXTI_LINE_7) )
    {
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
    }
}
/*------------------------------------------------
* @function :M2霍尔中断
* @input    :请填写
* @output   :请填写
* @explain  :请填写
* @author   :忘川
* @date     :2022/12/22
------------------------------------------------*/
void EXTI15_10_IRQHandler(void)
{
    if ( LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_10|LL_EXTI_LINE_11|LL_EXTI_LINE_12) )
    {
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
        HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
    }
}

