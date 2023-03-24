/*
*******************************************************************************
 *��    Ȩ�� 2021-xxxx,  GaussianRobot
 *�� �� ���� stm32f4xx.mc.it.c�жϺ���
 *��    �飺 ���ڴ��������жϺ���
 *��    �ߣ� LMmotor\����
 *��    �ڣ� 2022.12.6
 *����������
*******************************************************************************
 *��ע��
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


s16 HALL_CH1_Value1=0,HALL_CH1_Value2=0,HALL_CH1_ValueDelta=0,HALL_CH1_ValueDelta1,HALL_CH1_ValueDelta2,HALL_CH1_ValueOffset=0;     /*PWM�����ʼ��λ������δʹ�õ�����*/
s16 HALL_CH2_Value1=0,HALL_CH2_Value2=0,HALL_CH2_ValueDelta=0,HALL_CH2_ValueDelta1,HALL_CH2_ValueDelta2,HALL_CH2_ValueOffset=0;
s16 PWM_Init_Electrical_Angle=0,PWM_Init_Electrical_Angle2=0;                                                                       /*ͬ�ϣ�PWM��λ�õ��ĳ�ʼ�Ƕ�*/

uint8_t debug_motor1_exit= 0;                                 /*���ڲ���ʧ������*/
uint8_t debug_motor2_exit= 0;                                 /*���ڲ���ʧ������*/
int16_t debug_motor1_Angle_Compensation = 0;                  /*���ڲ���ʧ������*/
int16_t debug_motor2_Angle_Compensation = 0;                  /*���ڲ���ʧ������*/
#define   BrakeFilterDeep  5                                  /*brake�ж��˲�����*/
uint16_t BrakeFiltersCnt[2] = {0};                            /*brake�ж��˲��ƴ�*/

extern IWDG_HandleTypeDef hiwdg;
extern STM_Handle_t STM[NBR_OF_MOTORS];
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef husart_debug;
extern PWMC_Handle_t * pwmcHandle[NBR_OF_MOTORS];

u16 EMG_State,EMG_State2;                                     /*��ͣ*/
u32 led_1ms_cnt = 0;                                          /*LEDָʾ�Ƽ���*/

u8 datalens = 0;                                              /*���ڽ��յ����ݳ���*/
extern u8 g_uart_recv_ok;                                     /*���ڽ�����ɱ�־*/
#define Labview_uart_Flag1CNT   2                             /*����Labview_uart��ӡ��������*/
uint8_t Labview_uart_Flag1 = 0;                               /*����Labview_uart��ӡѭ�����ڼ���*/
uint16_t CNT_1MS[MAX_AXES];                                   /*1ms����*/

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

uint8_t PrevHallState1,PrevHallState2,HallState1,HallState2;                   /*���ݻ���״̬�ı任ȷ����ʼ�Ƕȸ���*/
int16_t HAL_Init_Electrical_Angle2 = 0,HAL_CommutationAngle2 = 0;
int16_t HAL_Init_Electrical_Angle = 0,HAL_CommutationAngle = 0;

u16 Motor1HallGetTimCNT[200],Motor2HallGetTimCNT[200];                         /*Ϊ�˲鿴��ʱ��CNTֵ������*/
s16 Hall_ElAngle1[7],Hall_ElAngle2[7];                                         /*����ѧϰʱʹ�ü�¼ÿ������״̬�ĽǶ�ֵ*/

extern s16 ENC_ElAngle,ENC_ElAngle2;
extern int32_t hSpeedRef_Pos;
/*------------------------------------------------
* @function :ADC�жϺ���,ע��ͨ���ĵ�������
* @input    :
* @output   :
* @explain  :tim1(100us)��tim8(100us)��ch4��������������50us
* @author   :wangchuan
* @date     :2022/12/22
------------------------------------------------*/
void ADC_IRQHandler(void)
{
//   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);                        /*���ڲ���ADCִ�����ڣ�����*/
    if(LL_ADC_IsActiveFlag_JEOS(ADC1))
    {
        ADC1->SR &= ~(uint32_t)(LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JSTRT);          /*����жϱ�־λ*/
        /*��Ƶ���񣬻�ȡ��ǶȲ��ҽ���FOC����*/
        TSK_HighFrequencyTask();                                                /*GUI, this section is present only if DAC is disabled*/
    }
#ifdef ADC3
    else
    {
        ADC3->SR &= ~(uint32_t)(LL_ADC_FLAG_JEOS | LL_ADC_FLAG_JSTRT);          /*����жϱ�־λ*/
        /*��Ƶ���񣬻�ȡ��ǶȲ��ҽ���FOC����*/
        TSK_HighFrequencyTask();                                                /*GUI, this section is present only if DAC is disabled*/
    }
#endif
    CanLoadRate.timer++;
    if(RPDO_SYNC_SIGN & RPDO_SYNC_READY)
    {
        RPDO_SYNC_SIGN &= ~RPDO_SYNC_READY;
        _RPOD_SyncEvent(&CANopen_Drive,pRpdoDirectPar,1);
    }

//  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);                       /*���ڲ���ADCִ�����ڣ�����*/
}

/*------------------------------------------------
* @function :TIM1�����ж�
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
* @function :TIM8�����ж�
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
* @function :M1ɲ���ж�
* @input    :
* @output   :
* @explain  :��ʱ�������õ�PWM�Զ����ʹ�ܹ��ܣ�brake�ܵ����ſ����˳�
             ���ǲ�Ӱ��Ӳ����������Ϊ�ڽ����ж�֮ǰӲ���Ѿ��ر�PWM���
             ��һ�����ڼ�������� BrakeFilterDeep �˲�5��
             BrakeFiltersCnt ÿ100ms���һ��
* @author   :wangchuan
* @date     :2022/12/22
------------------------------------------------*/
void TIMx_BRK_M1_IRQHandler(void)
{
    BrakeFiltersCnt[M1]++;
    if(BrakeFiltersCnt[M1] > BrakeFilterDeep)
    {
        R3_2_SwitchOffPWM( pwmcHandle[M1] );                                 /*�ر��Զ����ʹ�ܲ��ҹر�����PWM���*/
        if (LL_TIM_IsActiveFlag_BRK(PWM_Handle_M1.pParams_str->TIMx))
        {
            LL_TIM_ClearFlag_BRK(PWM_Handle_M1.pParams_str->TIMx);
            R3_2_BRK_IRQHandler(&PWM_Handle_M1);                             /*��brake����*/
        }
        /* Systick is not executed due low priority so is necessary to call MC_Scheduler here.*/
//        MC_Scheduler();//���������ı���static volatile uint16_t hMFTaskCounterM1 = 0; ���Ͳ��ʺϽ��������ļ��ĵ��ã�������������
    }
    else
    {
        LL_TIM_ClearFlag_BRK(PWM_Handle_M1.pParams_str->TIMx);
    }
}
/*------------------------------------------------
* @function :M2ɲ���ж�
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
        R3_2_SwitchOffPWM( pwmcHandle[M2] );                                /*�ر��Զ����ʹ�ܲ��ҹر�����PWM���*/
        if (LL_TIM_IsActiveFlag_BRK(PWM_Handle_M2.pParams_str->TIMx))
        {
            LL_TIM_ClearFlag_BRK(PWM_Handle_M2.pParams_str->TIMx);
            R3_2_BRK_IRQHandler(&PWM_Handle_M2);                            /*��brake����*/
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
* @function :�����ж�
* @input    :
* @output   :
* @explain  :
* @author   :wangchuan
* @date     :2022/12/22
------------------------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if((GPIO_Pin == M1_HALL_U_Pin )||(GPIO_Pin == M1_HALL_V_Pin )||(GPIO_Pin == M1_HALL_W_Pin ))     /*M1��������ж�*/
    {
        HALL_CC_First++;                                                                             /*��������жϵĴ���*/
        PrevHallState1 = HallState1;
        HallState1 = HALL_GetPhase1();                                                               /*��ȡ��ǰ����״̬*/
        if(HallStudyFlag1 == 1)                                                                      /*����ѧϰ*/
        {
            if(HallStudyCNT<Hall_EAC_Ticks)                                                          /*3��*/
            {
                Angle_Switch = 0;
            }
            else
            {
                Hall_ElAngle1[HallState1] = ENC_ElAngle;                                             /*��ȡ��Ƕ�*/
                if(HallState1 == 5)                                                                  /*�����Ƕ�ת��һ�ܣ��൱��1�Լ�*/
                {
                    Hall_AvElAngle1CNT++;
                    Hall_AvElAngle1Sum += Hall_ElAngle1[5];                                          /*��Ƕ��ۼӺ�*/
                }
            }
        }
        else
        {

            HALL_CC_First11++;                                                                        /*����*/
            Hall_AvElAngle1CNT =0;                                                                    /*��Ƕ�����*/
            Hall_AvElAngle1Sum =0;                                                                    /*����ѧϰ��ǶȺ�����*/
            Motor1HallGetTimCNT[HALL_CC_First11] = LL_TIM_GetCounter ( TIM4 );                        /*Ϊ�˲鿴��ʱ��CNTֵ������*/
            if(HALL_CC_First11>=99)                                                                   /*����*/
            {
                HALL_CC_First11=0;
            }

            if(HALL_CC_First<2)                                                                        /*�״��������У׼���λ��*/
            {
                switch ( HallState1 )                                                                  /*���ݻ���״̬����жϵ������λ��*/
                {
                case STATE_5:
                    if ( PrevHallState1 == STATE_4 )                                                   /*����״̬��4->5*/
                    {
//						HAL_CommutationAngle1 = (int16_t)(HALL_PHASE_SHIFT1 * 65536/360);//
                        HAL_CommutationAngle = (int16_t)(HALL_PHASE_SHIFT);
                    }
                    else if ( PrevHallState1 == STATE_1 )                                              /*����״̬��1->5*/
                    {
                        HAL_CommutationAngle = ( int16_t )( (int16_t)(HALL_PHASE_SHIFT) + S16_60_PHASE_SHIFT );
                    }
                    else
                    {
                        if(HALL_CC_First > 0)             /*HALL_CC_First ��uint8_t ���ͣ����Ϊ0��--���Ǿͻ�ֱ�ӱ��255*/
                        {
                            HALL_CC_First--;                                                               /*�������˴�˵�����ζ�λ������Ҫ���¶�λ*/
                        }
                        debug_motor1_exit = 5;                                                         /*���ڲ���ʧ������*/
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
                        if(HALL_CC_First > 0)             /*HALL_CC_First ��uint8_t ���ͣ����Ϊ0��--���Ǿͻ�ֱ�ӱ��255*/
                        {
                            HALL_CC_First--;                 /*�������˴�˵�����ζ�λ������Ҫ���¶�λ*/
                        }
                        debug_motor1_exit = 1;//���ڲ���ʧ������
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
                        if(HALL_CC_First > 0)                                      /*HALL_CC_First ��uint8_t ���ͣ����Ϊ0��--���Ǿͻ�ֱ�ӱ��255*/
                        {
                            HALL_CC_First--;                                       /*�������˴�˵�����ζ�λ������Ҫ���¶�λ*/
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
                        if(HALL_CC_First > 0)                                        /*HALL_CC_First ��uint8_t ���ͣ����Ϊ0��--���Ǿͻ�ֱ�ӱ��255*/
                        {
                            HALL_CC_First--;                                         /*�������˴�˵�����ζ�λ������Ҫ���¶�λ*/
                        }
                        debug_motor1_exit = 2;//���ڲ���ʧ������
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
                        if(HALL_CC_First > 0)                                         /*HALL_CC_First ��uint8_t ���ͣ����Ϊ0��--���Ǿͻ�ֱ�ӱ��255*/
                        {
                            HALL_CC_First--;                                          /*�������˴�˵�����ζ�λ������Ҫ���¶�λ*/
                        }
                        debug_motor1_exit = 6;//���ڲ���ʧ������
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
                        if(HALL_CC_First > 0)                                            /*HALL_CC_First ��uint8_t ���ͣ����Ϊ0��--���Ǿͻ�ֱ�ӱ��255*/
                        {
                            HALL_CC_First--;                                             /*�������˴�˵�����ζ�λ������Ҫ���¶�λ*/
                        }
                        debug_motor1_exit = 4;//���ڲ���ʧ������
                    }
                    break;
                default:
                    if(HALL_CC_First > 0)                                                /*HALL_CC_First ��uint8_t ���ͣ����Ϊ0��--���Ǿͻ�ֱ�ӱ��255*/
                    {
                        HALL_CC_First--;                                                 /*�������˴�˵�����ζ�λ������Ҫ���¶�λ*/
                    }
                    break;
                }

                if(HallStudyFlag1 == 0)                                                                                   /*��������л���ѧϰ������㻻���*/
                {
                    if(HALL_CC_First == 1 && HAL_CommutationAngle != 0)                                                   /*���� HAL_CommutationAngle ��= 0 û���ų�������λ�����HAL_CommutationAngle = 0���������������ܷ�*/
                    {
                        ENCODER_M1.Angle_Compensation   = HAL_CommutationAngle - ENCODER_M1._Super.hElAngle ;
                        debug_motor1_Angle_Compensation = HAL_CommutationAngle - ENCODER_M1._Super.hElAngle ;             /*���ڲ鿴����ֵ*/
                    }
                }
                else                                                                                                      /*������л���ѧϰ���������*/
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

    if((GPIO_Pin == M2_HALL_U_Pin )||(GPIO_Pin == M2_HALL_V_Pin )||(GPIO_Pin == M2_HALL_W_Pin ))              /*M2��������ж�*/
    {
        HALL2_CC_First++;
        PrevHallState2 = HallState2;//�����жϷ���
        HallState2 = HALL_GetPhase2();;

        if(HallStudyFlag2 == 1)//����ѧϰ
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
* @function :TIM4�Ƚ��жϣ�M1������
* @input    :
* @output   :
* @explain  :���ڱ�����PWMģʽ������г�ʼ��λ��S02A11-C3����汾δʹ���ж��Ѿ��ر�
* @author   :wangchuan
* @date     :2022/12/22
------------------------------------------------*/
void SPD_TIM_M1_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_CC1 (TIM4))                                /*TIM4 CH1ͨ���Ƚ��ж�*/
    {
        HallState1 = (GPIOD->IDR>>12)&(0x1);                           /*ΪʲôҪ��ȡ������A�ĵ�ƽ״̬*/
        if(HallState1 == 1)
        {
//			TIM4->CNT = 0;
            HALL_CH1_Value1 = LL_TIM_GetCounter ( TIM4 );              /*��ȡ����������ֵ*/
//			TIM4->CCER |=0x0002 ;
        }
        else
        {
            HALL_CH1_Value2 = LL_TIM_GetCounter ( TIM4 );              /*��ȡ����������ֵ*/
            if(HALL_CH1_Value2>HALL_CH1_Value1)                        /*����������ֵû�����*/
            {
                HALL_CH1_ValueDelta = HALL_CH1_Value2 - HALL_CH1_Value1 ;
            }
            else                                                       /*�������������ֵ���*/
            {
                HALL_CH1_ValueDelta = HALL_CH1_Value2 + 65536- HALL_CH1_Value1 ;
            }
            if(HALL_CH1_ValueDelta>111)                                                        /*��������������������*/
            {
                if(HALL_CH1_ValueDelta>29000)
                {
                    HALL_CH1_ValueDelta = 29000;
                }

                HALL_CH1_ValueDelta1 = 29000 - HALL_CH1_ValueDelta;
                HALL_CH1_ValueDelta1 = (HALL_CH1_ValueDelta1%2063) ;                           /*��������������������*/
                HALL_CH1_ValueDelta2 = (HALL_CH1_ValueDelta1 - HALL_CH1_ValueOffset)*31.76;    /*��������������������*/

                if(HALL_CC_First<1 )
                {
                    PWM_Init_Electrical_Angle =  HALL_CH1_ValueDelta2;                         /*��Ҫ*/
                    HALL_CC_First++;
                }
            }
            else
            {
                HALL_CH1_ValueDelta1 = 0;
            }
//			TIM4->CCER &=0xfffd ;
        }
        LL_TIM_ClearFlag_CC1(TIM4);                                                            /*���TIM4CC1�Ƚ��ж�*/
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
* @function :TIM2�����ж�
* @input    :
* @output   :
* @explain  :����CNT�����������δʹ�ã��ж��Ѿ��ر�
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
* @function :���ڽ����ж�
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
        __HAL_UART_CLEAR_IDLEFLAG(&husart_debug);                          /*���uart3�����жϱ�־λ*/
        HAL_UART_DMAStop(&husart_debug);                                   /*ֹͣuart3��DMA����*/
        datalens =50 - husart_debug.hdmarx->Instance->NDTR;                /*�õ�DMA�������ݵĳ���*/
        if(datalens < 50)                                                  /*���ݳ���û�г���50��*/
        {
            if(UsartRxBuffer[0] == 0xAA && (UsartRxBuffer[datalens-1] == 0x2F) && UsartRxBuffer[datalens-2] == datalens) /*�ж�֡ͷ��֡β*/
            {
                g_uart_recv_ok =1;                                         /*�������һ�����ݣ���Usart_Labview_Analyze�����п�������*/
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
* @function :Ӳ���ж�
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
* @function :�δ�ʱ���ж�
* @input    :����д
* @output   :����д
* @explain  :500usִ��һ��
* @author   :����
* @date     :2022/12/22
------------------------------------------------*/
void SysTick_Handler(void)
{
    static uint8_t SystickDividerCounter = SYSTICK_DIVIDER;
    if (SystickDividerCounter == SYSTICK_DIVIDER)  /*1ms����*/
    {
        SystickDividerCounter = 0;                         /*2��Ƶ����*/
        led_1ms_cnt++;                                     /*������ʱδ��0��4294967295ms/1000=(s)/60=(min)/60=(h)/24=49(Day)*/
        Usart_Labview_Analyze();                           /*����labviewָ�����*/
        Damping_Of_Motor();                                /*����ģʽ*/

        if(led_1ms_cnt%10==0)                              /*10msִ��LEDָʾ����˸*/
        {
            Display_error();                               /*������������Ĵ���һ��32λ��������*/
            DisplayErrorLed_Handle();                      /*����ָʾ��*/
        }

        Sci_Monitor();                                     /*USART3�������ݴ�����*/
        Sci_Stack_Monitor(&USART3_EMAIL.State_monitor);    /*USART3���ڶ��д�����*/

        if(led_1ms_cnt%100 == 0)                           /*100ms���һ��brake�˲�����ֵ,����ι��һ��*/
        {
            HAL_IWDG_Refresh(&hiwdg);
            Clear_Brake_Filters_Cnt();
        }

        M1M2_Initial_Position();                           /*�����ʼλ��ѧϰ*/

        CNT_1MS[M1]++;
        CNT_1MS[M2]++;
        HallStudyCNT++;
        HallStudyCNT2++;
        Labview_uart_Flag ++;
        Labview_uart_Flag1++;
        HAL_IncTick();                                      /*�δ�ʱ��1ms����*/
        HAL_SYSTICK_IRQHandler();                           /*�����ݿ���ɾ��*/

    }
    else
    {

    }
    SystickDividerCounter ++;
    ProceedDriveStateChange();                              /*402״̬��*/
    ProceedDriverHandler();                                 /*402״̬��*/
    PendingOptionCode();                                    /*402״̬��*/
    MC_RunMotorControlTasks();//500us
}
/*------------------------------------------------
* @function :�ⲿ��ͣ�ж�
* @input    :����д
* @output   :����д
* @explain  :����д
* @author   :����
* @date     :2022/12/22
------------------------------------------------*/
void EXTI0_IRQHandler (void)
{
    if ( LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_0) )
    {
        EMG_State = (Start_Stop_GPIO_Port->IDR>>0)&(0x01);        /*��ȡ��ǰ����״̬*/
        if(EMG_State == STOPISON)
        {
            for(u16 i=0; i<10000; i++);                           /*�൱���˲�*/
            EMG_State2 = (Start_Stop_GPIO_Port->IDR>>0)&(0x01);
            EMG_State =  (Start_Stop_GPIO_Port->IDR>>0)&(0x01);
            if(EMG_State == STOPISON)                                    /*��ͣ������*/
            {
                pCtrlPar[M1].SetVelMotor = 0;
                pCtrlPar[M2].SetVelMotor = 0;
                pCtrlPar[M1].M1M2_VelSet = 0;
            }
        }
        LL_EXTI_ClearFlag_0_31 (LL_EXTI_LINE_0);                   /*����жϱ�־*/
    }
}
/*------------------------------------------------
* @function :M1����
* @input    :����д
* @output   :����д
* @explain  :����д
* @author   :����
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
* @function :M2�����ж�
* @input    :����д
* @output   :����д
* @explain  :����д
* @author   :����
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

