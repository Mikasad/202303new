/**
  ******************************************************************************

  ******************************************************************************

  ******************************************************************************
  */
/* °ü???????? ----------------------------------------------------------------*/
#include "bsp_BDCMotor.h"
#include "main.h"
/* ?????à???¨?? --------------------------------------------------------------*/
/* ?????ê?¨?? ----------------------------------------------------------------*/
/* ????±??? ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx_BDCMOTOR;
MotorControlParameters_t MotorControl[MOTOR_NUM];
u16 Motor_Prescaler = 83;
HALL_Study_t HALL_Study[2];
int8_t  Motor_Type=0;         //滚刷电机选择，0：联谊滚刷电机   1：和泰滚刷电机
//__IO int32_t PWM_Duty=BDCMOTOR_DUTY_ZERO;         // ????±???PWM_Duty/BDCMOTOR_TIM_PERIOD*100%
/* ????±??? ------------------------------------------------------------------*/
/* ???????????? --------------------------------------------------------------*/
/* ?????? --------------------------------------------------------------------*/

//MotorControl[6] =
//{
//	.Speed_Set = 1000,      //
//
//  .SpeedLimit = 4000,

//	.PWM_Duty = 0,
//	.Pole_Paires = 2,

//}
/**
  * ????????: ?è?????ú????
  * ????????: Duty,????????????±?
  * ?? ?? ??: ??
  * ??    ?÷: ??
  */


int test6 = 0;

void SetMotorSpeed(uint8_t Motor_NUM,int16_t PWM_Duty)    //启动电机
{
    if(PWM_Duty > PWM_PERIOD)
    {
        PWM_Duty = PWM_PERIOD ;
    }
    else if(PWM_Duty <- PWM_PERIOD)
    {
        PWM_Duty = -PWM_PERIOD ;
    }
    switch ( Motor_NUM )
    {
    case 0:

        if(MotorControl[0].PWM_DutySet > 0)  //正转
        {
            MotorControl[0].Direction = CW;
            if(MotorControl[0].PWM_DutySet > PERCENT_95_OF_PWM_PERIOD)
            {
                MotorControl[0].PWM_DutySet = PERCENT_95_OF_PWM_PERIOD;
            }
        }
        else if(MotorControl[0].PWM_DutySet < 0)  //反转
        {
            MotorControl[0].Direction = CCW;
            if(MotorControl[0].PWM_DutySet < -PERCENT_95_OF_PWM_PERIOD)
            {
                MotorControl[0].PWM_DutySet = -PERCENT_95_OF_PWM_PERIOD;
            }
        }
        else
        {
            MotorControl[0].Direction = STOPSTOP;
        }
        switch(MotorControl[0].Direction)
        {
        case CW:
            if(MotorControl[0].Direction != MotorControl[0].LastMotorDirection)
            {
                MotorControl[0].Pole_Paires++;
                if(MotorControl[0].Pole_Paires > 200)
                {
                    MotorControl[0].Pole_Paires = 0;
                    MotorControl[0].LastMotorDirection = MotorControl[0].Direction;
                    HAL_GPIO_WritePin(GPIOD,PWM_AL1_Pin,GPIO_PIN_SET);
                }
                else
                {
//                    MOTOR0STOP();
                    TIM2->CCR1 = 0;
                    TIM2->CCR2 = 0;
                    HAL_GPIO_WritePin(GPIOD,PWM_AL1_Pin,GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOD,PWM_BL1_Pin,GPIO_PIN_SET);
                    MotorControl[0].PWM_Duty = 0;

                }
            }
            else
            {
                Ramp_PPWM(0);                  //电机0按照加速度运行
                MOTOR0CW();
            }
            break;

        case CCW:
            if(MotorControl[0].Direction != MotorControl[0].LastMotorDirection)
            {
                MotorControl[0].Pole_Paires++;
                if(MotorControl[0].Pole_Paires > 200)
                {
                    MotorControl[0].Pole_Paires = 0;
                    MotorControl[0].LastMotorDirection = MotorControl[0].Direction;
                    HAL_GPIO_WritePin(GPIOD,PWM_BL1_Pin,GPIO_PIN_SET);
                }
                else
                {
//                    MOTOR0STOP();
                    TIM2->CCR1 = 0;
                    TIM2->CCR2 = 0;
                    HAL_GPIO_WritePin(GPIOD,PWM_AL1_Pin,GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOD,PWM_BL1_Pin,GPIO_PIN_RESET);
                    MotorControl[0].PWM_Duty = 0;
                }
            }
            else
            {
                Ramp_PPWM(0);
                MOTOR0CCW();
            }
            break;
        case STOPSTOP:
            MOTOR0STOP();
            break;
        }

        break;

    case 1:
//        if(PWM_Duty > 0)
//        {
//            TIM2->CCR3 = PWM_Duty;
//            TIM2->CCR4 = 0;
//						HAL_GPIO_WritePin(GPIOD,PWM_AL2_Pin,GPIO_PIN_SET);//PD2 SET
//						HAL_GPIO_WritePin(GPIOD,PWM_BL2_Pin,GPIO_PIN_RESET);//PD3 RESET
//				}
//        else
//        {
//            TIM2->CCR3 = 0;
//            TIM2->CCR4 =  -PWM_Duty;
//					  HAL_GPIO_WritePin(GPIOD,PWM_BL2_Pin,GPIO_PIN_SET);//PD3 SET
//						HAL_GPIO_WritePin(GPIOD,PWM_AL2_Pin,GPIO_PIN_RESET);//PD2 RESET
//        }
        if(MotorControl[1].PWM_DutySet > 0)  //正转
        {
            MotorControl[1].Direction = CW;
            if(MotorControl[1].PWM_DutySet > PERCENT_95_OF_PWM_PERIOD)
            {
                MotorControl[1].PWM_DutySet = PERCENT_95_OF_PWM_PERIOD;
            }
        }
        else if(MotorControl[1].PWM_DutySet < 0)  //反转
        {
            MotorControl[1].Direction = CCW;
            if(MotorControl[1].PWM_DutySet < -PERCENT_95_OF_PWM_PERIOD)
            {
                MotorControl[1].PWM_DutySet = -PERCENT_95_OF_PWM_PERIOD;
            }
        }
        else
        {
            MotorControl[1].Direction = STOPSTOP;
        }
        switch(MotorControl[1].Direction)
        {
        case CW:
            if(MotorControl[1].Direction != MotorControl[1].LastMotorDirection)
            {
                MotorControl[1].Pole_Paires++;
                if(MotorControl[1].Pole_Paires > 200)
                {
                    MotorControl[1].Pole_Paires = 0;
                    MotorControl[1].LastMotorDirection = MotorControl[1].Direction;
                    HAL_GPIO_WritePin(GPIOD,PWM_AL2_Pin,GPIO_PIN_SET);
                }
                else
                {
//                    MOTOR1STOP();
                    TIM2->CCR3 = 0;
                    TIM2->CCR4 = 0;
                    HAL_GPIO_WritePin(GPIOD,PWM_AL2_Pin,GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOD,PWM_BL2_Pin,GPIO_PIN_SET);
                    MotorControl[1].PWM_Duty = 0;
                }
            }
            else
            {
                Ramp_PPWM(1);                  //电机1按照加速度运行
                MOTOR1CW();
            }
            break;

        case CCW:
            if(MotorControl[1].Direction != MotorControl[1].LastMotorDirection)
            {
                MotorControl[1].Pole_Paires++;
                if(MotorControl[1].Pole_Paires > 200)
                {
                    MotorControl[1].Pole_Paires = 0;
                    MotorControl[1].LastMotorDirection = MotorControl[1].Direction;
                    HAL_GPIO_WritePin(GPIOD,PWM_BL2_Pin,GPIO_PIN_SET);
                }
                else
                {
//                    MOTOR1STOP();
                    TIM2->CCR3 = 0;
                    TIM2->CCR4 = 0;
                    HAL_GPIO_WritePin(GPIOD,PWM_AL2_Pin,GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOD,PWM_BL2_Pin,GPIO_PIN_RESET);
                    MotorControl[1].PWM_Duty = 0;
                }
            }
            else
            {
                Ramp_PPWM(1);
                MOTOR1CCW();
            }
            break;
        case STOPSTOP:
            MOTOR1STOP();
            break;
        }

        break;


    case 2:
        if(MotorControl[2].PWM_DutySet > 0)  //正转
        {
            MotorControl[2].Direction = CW;
            if(MotorControl[2].PWM_DutySet > PERCENT_95_OF_PWM_PERIOD)
            {
                MotorControl[2].PWM_DutySet = PERCENT_95_OF_PWM_PERIOD;
            }
        }
        else if(MotorControl[2].PWM_DutySet < 0)  //反转
        {
            MotorControl[2].Direction = CCW;
            if(MotorControl[2].PWM_DutySet < -PERCENT_95_OF_PWM_PERIOD)
            {
                MotorControl[2].PWM_DutySet = -PERCENT_95_OF_PWM_PERIOD;
            }
        }
        else
        {
            MotorControl[2].Direction = STOPSTOP;
        }
        switch(MotorControl[2].Direction)
        {
        case CW:
            if(MotorControl[2].Direction != MotorControl[2].LastMotorDirection)
            {
                MotorControl[2].Pole_Paires++;
                if(MotorControl[2].Pole_Paires > 200)
                {
                    MotorControl[2].Pole_Paires = 0;
                    MotorControl[2].LastMotorDirection = MotorControl[2].Direction;
                    HAL_GPIO_WritePin(GPIOD,PWM_AL3_Pin,GPIO_PIN_SET); //电已经充好，2ms之后将进入MOTOR2CW，在这先关闭需要打开的上管所对应的下管
                }
                else
                {
//                    MOTOR2STOP();
                    TIM4->CCR1 = 0;
                    TIM4->CCR2 = 0;
                    HAL_GPIO_WritePin(GPIOD,PWM_AL3_Pin,GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOD,PWM_BL3_Pin,GPIO_PIN_SET);
                    MotorControl[2].PWM_Duty = 0;
                }
            }
            else
            {
                Ramp_PPWM(2);                  //电机1按照加速度运行
                MOTOR2CW();
            }
            break;

        case CCW:
            if(MotorControl[2].Direction != MotorControl[2].LastMotorDirection)
            {
                MotorControl[2].Pole_Paires++;
                if(MotorControl[2].Pole_Paires > 200)
                {
                    MotorControl[2].Pole_Paires = 0;
                    MotorControl[2].LastMotorDirection = MotorControl[2].Direction;
                    HAL_GPIO_WritePin(GPIOD,PWM_BL3_Pin,GPIO_PIN_SET);
                }
                else
                {
//                    MOTOR2STOP();
                    TIM4->CCR1 = 0;
                    TIM4->CCR2 = 0;
                    HAL_GPIO_WritePin(GPIOD,PWM_AL3_Pin,GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOD,PWM_BL3_Pin,GPIO_PIN_RESET);
                    MotorControl[2].PWM_Duty = 0;
                }
            }
            else
            {
                Ramp_PPWM(2);
                MOTOR2CCW();
            }
            break;
        case STOPSTOP:
            MOTOR2STOP();
            break;
        }

        break;

    case 3:

        Ramp_PPWM(3);
        if(PWM_Duty > 0)
        {
            TIM10->CCR1 = 2*(PWM_PERIOD - MotorControl[3].PWM_Duty);
        }
        else if(PWM_Duty<0)
        {
            TIM10->CCR1  = 2*(PWM_PERIOD + MotorControl[3].PWM_Duty);
        }
				else 
        {
            TIM10->CCR1  = 2*(PWM_PERIOD + PWM_PeriodOFFSET);
        }

        break;

    case 4:

        Ramp_PPWM(4);
        if(PWM_Duty > 0)
        {
            TIM11->CCR1 = 2*(PWM_PERIOD - MotorControl[4].PWM_Duty);
        }
        else if(PWM_Duty < 0)
        {
            TIM11->CCR1  = 2*(PWM_PERIOD + MotorControl[4].PWM_Duty);
        }
				else 
        {
            TIM11->CCR1  = 2*(PWM_PERIOD +PWM_PeriodOFFSET);
        }

        break;

    case 5:


        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
        if(MotorControl[5].Speed_Ref < 0)
        {
            MotorControl[5].Hall.HallState_CCW = 0x07 ^ MotorControl[5].Hall.HallState;
            BLDC1_PhaseChange( MotorControl[5].Hall.HallState_CCW,PWM_Duty);
        }
        else if(MotorControl[5].Speed_Ref > 0)
        {

            MotorControl[5].Hall.HallState_CW = MotorControl[5].Hall.HallState;
            BLDC1_PhaseChange( MotorControl[5].Hall.HallState_CW,PWM_Duty);
        }
        else
        {
            PWM_Duty =0;
            MotorControl[5].Hall.HallState_CW = MotorControl[5].Hall.HallState;
            BLDC1_PhaseChange( MotorControl[5].Hall.HallState_CW,PWM_Duty);
			PID_Speed_InitStruct[0].wIntegral = 0;
			PID_Current_InitStructure[0].wIntegral = 0;
        }
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

        break;

    case 6:

        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
        if(MotorControl[6].Speed_Ref < 0)
        {
            MotorControl[6].Hall.HallState_CCW = 0x07 ^ MotorControl[6].Hall.HallState;
            BLDC2_PhaseChange( MotorControl[6].Hall.HallState_CCW,PWM_Duty);
        }
        else if(MotorControl[6].Speed_Ref > 0)
        {
            MotorControl[6].Hall.HallState_CW = MotorControl[6].Hall.HallState;
            BLDC2_PhaseChange( MotorControl[6].Hall.HallState_CW,PWM_Duty);
        }
        else
        {
            PWM_Duty =0;
            MotorControl[6].Hall.HallState_CW = MotorControl[6].Hall.HallState;
            BLDC2_PhaseChange( MotorControl[6].Hall.HallState_CW,PWM_Duty);
			PID_Speed_InitStruct[1].wIntegral = 0;
			PID_Current_InitStructure[1].wIntegral = 0;
        }
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

        break;

    case 7:    				//左边刷																																		  															//边刷

        Ramp_PPWM(7);
        if(PWM_Duty > 0)
        {
            TIM3->CCR3 = PWM_PERIOD - MotorControl[7].PWM_Duty;
        }
        else if(PWM_Duty < 0)
        {
            TIM3->CCR3 = PWM_PERIOD + MotorControl[7].PWM_Duty;
        }
				else 
        {
            TIM3->CCR3 = PWM_PERIOD + PWM_PeriodOFFSET;
        }
//        TIM3->CCR3 = 0;
				break;



    case 8:    		  //右边刷

        Ramp_PPWM(8);
        if(PWM_Duty > 0)
        {
            TIM3->CCR4 = PWM_PERIOD - MotorControl[8].PWM_Duty;
        }
        else if(PWM_Duty < 0)
//					else 
        {
            TIM3->CCR4 = PWM_PERIOD + MotorControl[8].PWM_Duty;
        }
				else 
        {
            TIM3->CCR4 = PWM_PERIOD + PWM_PeriodOFFSET;
        }
        break;


    case 9:
        if(MotorControl[9].PWM_DutySet > 0)  //正转
        {
            MotorControl[9].Direction = CW;
            if(MotorControl[9].PWM_DutySet > PERCENT_95_OF_PWM_PERIOD)
            {
                MotorControl[9].PWM_DutySet = PERCENT_95_OF_PWM_PERIOD;
            }
        }
        else if(MotorControl[9].PWM_DutySet < 0)  //反转
        {
            MotorControl[9].Direction = CCW;
            if(MotorControl[9].PWM_DutySet < -PERCENT_95_OF_PWM_PERIOD)
            {
                MotorControl[9].PWM_DutySet = -PERCENT_95_OF_PWM_PERIOD;
            }
        }
        else
        {
            MotorControl[9].Direction = STOPSTOP;
        }
        switch(MotorControl[9].Direction)
        {
        case CW:
            if(MotorControl[9].Direction != MotorControl[9].LastMotorDirection)
            {
                MotorControl[9].Pole_Paires++;
                if(MotorControl[9].Pole_Paires > 200)
                {
                    MotorControl[9].Pole_Paires = 0;
                    MotorControl[9].LastMotorDirection = MotorControl[9].Direction;
                    HAL_GPIO_WritePin(GPIOD,PWM_AL4_Pin,GPIO_PIN_SET);
                }
                else
                {
//                    MOTOR9STOP();
                    TIM4->CCR3 = 0;
                    TIM4->CCR4 = 0;
                    HAL_GPIO_WritePin(GPIOD,PWM_AL4_Pin,GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOD,PWM_BL4_Pin,GPIO_PIN_SET);
                    MotorControl[9].PWM_Duty = 0;
                }
            }
            else
            {
                Ramp_PPWM(9);                  //电机1按照加速度运行
                MOTOR9CW();
            }
            break;

        case CCW:
            if(MotorControl[9].Direction != MotorControl[9].LastMotorDirection)
            {
                MotorControl[9].Pole_Paires++;
                if(MotorControl[9].Pole_Paires > 200)
                {
                    MotorControl[9].Pole_Paires = 0;
                    MotorControl[9].LastMotorDirection = MotorControl[9].Direction;
                    HAL_GPIO_WritePin(GPIOD,PWM_BL4_Pin,GPIO_PIN_SET);
                }
                else
                {
//                    MOTOR9STOP();
                    TIM4->CCR3 = 0;
                    TIM4->CCR4 = 0;
                    HAL_GPIO_WritePin(GPIOD,PWM_AL4_Pin,GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOD,PWM_BL4_Pin,GPIO_PIN_RESET);
                    MotorControl[9].PWM_Duty = 0;
                }
            }
            else
            {
                Ramp_PPWM(9);
                MOTOR9CCW();
            }
            break;
        case STOPSTOP:
            MOTOR9STOP();
            break;
        }

        break;

    case 10:  //电磁阀
        Ramp_PPWM(10);
        if(PWM_Duty > 0)
        {
            TIM3->CCR1 = PWM_PERIOD - MotorControl[10].PWM_Duty;
        }
        else if(PWM_Duty < 0)
        {
            TIM3->CCR1 = PWM_PERIOD + MotorControl[10].PWM_Duty;
        }
				else 
        {
            TIM3->CCR1 = PWM_PERIOD +PWM_PeriodOFFSET;
        }
//        TIM3->CCR1 = 0;

        break;
    case 11: //电动阀
        Ramp_PPWM(11);
        if(PWM_Duty > 0)
        {
            TIM3->CCR2 = PWM_PERIOD - MotorControl[11].PWM_Duty;
        }
        else if(PWM_Duty < 0)
        {
            TIM3->CCR2 = PWM_PERIOD + MotorControl[11].PWM_Duty;
        }
				else 
        {
            TIM3->CCR2 = PWM_PERIOD +PWM_PeriodOFFSET;
        }
//        TIM3->CCR2 = 0;
        break;
    case 12:         //单向有刷

        Ramp_PPWM(12);
        if(MotorControl[12].PWM_Duty < 0) MotorControl[12].PWM_Duty = -MotorControl[12].PWM_Duty;
        TIM8->CCR4 =  MotorControl[12].PWM_Duty;
        break;

        break;

    case 13: 	        //单向有刷
        Ramp_PPWM(13);
        if(MotorControl[13].PWM_Duty < 0) MotorControl[13].PWM_Duty = -MotorControl[13].PWM_Duty;
        TIM1->CCR4 = MotorControl[13].PWM_Duty;
        break;


    default:
        break;
    }

}



void SetMotorStop(uint8_t Motor_NUM)    //停止电机
{
    switch ( Motor_NUM )
    {
    case 0://???????ú0-???ò

        MOTOR0STOP();
        MotorControl[0].SpeedLimit = 1;
        break;

    case 1://???????ú1-???ò

        MOTOR1STOP();
        break;

    case 2://???????ú2-???ò

        MOTOR2STOP();
        break;

    case 3://???????ú3-???ò
        MotorControl[3].PWM_Duty = 0;
        TIM10->CCR1 = 2*(PWM_PERIOD+PWM_PeriodOFFSET) ;

        break;

    case 4: //???????ú4-???ò?????÷??
        MotorControl[4].PWM_Duty = 0;
        TIM11->CCR1 = 2*(PWM_PERIOD+PWM_PeriodOFFSET) ;


        break;

    case 5://???????ú5
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
        MotorControl[5].Hall.ChangeFlag = 1;
        MotorControl[5].PWM_Duty = 0;
        MotorControl[5].Speed_Ref = 0;
        PID_Speed_InitStruct[0].wIntegral = 0;
		PID_Current_InitStructure[0].wIntegral = 0;
        if(MotorControl[5].Speed_Real ==0 )
        {
            TIM1->CCER = OPEN_ALL_DOWN_TUBE;
        }
		    else
        {
            TIM1->CCER = CLOSE_ALL_TUBE;            //3月22日修复
        }
        break;

    case 6://???????ú6
        TIM8->CCR1 = 0;
        TIM8->CCR2 = 0;
        TIM8->CCR3 = 0;
        MotorControl[6].Hall.ChangeFlag = 1;
        MotorControl[6].PWM_Duty = 0;
        MotorControl[6].Speed_Ref = 0;
        PID_Speed_InitStruct[1].wIntegral = 0;
		PID_Current_InitStructure[1].wIntegral = 0;
        if(MotorControl[6].Speed_Real ==0 )
        {
            TIM8->CCER = OPEN_ALL_DOWN_TUBE;
        }
		    else
        {
            TIM8->CCER = CLOSE_ALL_TUBE;          //3月22日修复
        }
        break;

    case 7:

        TIM3->CCR3= PWM_PERIOD+PWM_PeriodOFFSET;
        MotorControl[7].PWM_Duty = 0;
        MotorControl[7].Speed_Real = 0;

        break;


    case 8:

        TIM3->CCR4= PWM_PERIOD+PWM_PeriodOFFSET;
        MotorControl[8].PWM_Duty = 0;
        MotorControl[8].Speed_Real = 0;
        break;

    case 9:
        MOTOR9STOP();
        break;
    case 10:
        MotorControl[10].PWM_Duty = 0;
        TIM3->CCR1= PWM_PERIOD+PWM_PeriodOFFSET;
        break;
    case 11:
        MotorControl[11].PWM_Duty = 0;
        TIM3->CCR2= PWM_PERIOD+PWM_PeriodOFFSET;
        break;
    case 12:

        TIM8->CCR4 = 0;
        MotorControl[12].PWM_Duty = 0;

        break;


    case 13:

        TIM1->CCR4 = 0;
        MotorControl[13].PWM_Duty = 0;

        break;

    default:
        break;
    }
}

/*------------------------------------------------
Function:BLDC1换向
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void BLDC1_PhaseChange(u8 bHallState, s16 PWM_Duty)

{
    if(PWM_Duty < 0) PWM_Duty = -PWM_Duty;
    if(bHallState == 0)
    {
        MC_SetFault(HALL5_SENSOR_ERR);
    }
    else if(bHallState == HALL_Study[0].HallTab[4])
    {
//        TIM1->CCER = 0x3c90;//TIM1->CCER = 0x1eba;//BC
        if( PWM_Duty< PWM_Complementary_Switch_M1 ) //重载控制方式改为互补模式，降低mos开关损耗，该测试未50机器加装散热片方案
        {
            TIM1->CCER = BtoC ;       //B->C
        }
        else
        {
            TIM1->CCER = B_complementary_to_C;       //B(complementary)->C
        }
        TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR2 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[0].HallTab[5])
    {
//        TIM1->CCER = 0x309c;//TIM1->CCER = 0x1abe;//BA
//        TIM1->CCR2 = PWM_Duty;
//        TIM1->CCR3 = PWM_Duty;
        if( PWM_Duty< PWM_Complementary_Switch_M1 )
        {
            TIM1->CCER = BtoA; //B->A
        }
        else
        {
            TIM1->CCER = B_complementary_to_A; //  //B(complementary)->A
        }
        TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR2 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[0].HallTab[0])
    {
//        TIM1->CCER = 0x390c;//  TIM1->CCER = 0x1bae;//CA
//        TIM1->CCR2 = PWM_Duty;
//        TIM1->CCR3 = PWM_Duty;
        if( PWM_Duty< PWM_Complementary_Switch_M1 )
        {
            TIM1->CCER = CtoA; //C->A
        }
        else
        {
            TIM1->CCER = C_complementary_to_A;  //C(complementary)->A
        }
        TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR3 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[0].HallTab[1])
    {
//        TIM1->CCER = 0x39c0;//TIM1->CCER = 0x1bea;//CB
//        TIM1->CCR1 = PWM_Duty;
//        TIM1->CCR3 = PWM_Duty;
        if( PWM_Duty< PWM_Complementary_Switch_M1 )
        {
            TIM1->CCER = CtoB    ; //C->B
        }
        else
        {
            TIM1->CCER = C_complementary_to_B; //C(complementary)->B
        }
        TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR3 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[0].HallTab[2])
    {
//       TIM1->CCER = 0x30c9;//TIM1->CCER = 0x1aeb;//AB
//        TIM1->CCR3 = PWM_Duty;
//        TIM1->CCR1 = PWM_Duty;
        if( PWM_Duty< PWM_Complementary_Switch_M1 )
        {
            TIM1->CCER = AtoB;  //A->B
        }
        else
        {
            TIM1->CCER = A_complementary_to_B ;  //A(complementary)->B
        }
        TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR1 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[0].HallTab[3])
    {
//        TIM1->CCER = 0x3c09;//TIM1->CCER = 0x1eab;//AC
        if( PWM_Duty< PWM_Complementary_Switch_M1 )
        {
            TIM1->CCER =  AtoC; //A->C
        }
        else
        {
            TIM1->CCER = A_complementary_to_C; //A(complementary)->C
        }
        TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR1 = PWM_Duty;
    }
    else if(bHallState == 7)
    {
        MC_SetFault(HALL5_SENSOR_ERR);
    }
}
/*------------------------------------------------
Function:BLDC2换向
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void BLDC2_PhaseChange(u8 bHallState, s16 PWM_Duty)
{
    if(PWM_Duty < 0) PWM_Duty = -PWM_Duty;
    if(bHallState == 0)
    {
        MC_SetFault(HALL6_SENSOR_ERR);
    }
    else if(bHallState == HALL_Study[1].HallTab[4])
    {
//        TIM8->CCER = 0x3c90;//TIM8->CCER = 0x3eba;//BC
        if( PWM_Duty< PWM_Complementary_Switch_M2)
        {
            TIM8->CCER = BtoC;
        }
        else
        {
            TIM8->CCER = B_complementary_to_C;
        }
        TIM8->CCR2 = PWM_PERIOD;
        TIM8->CCR1 = PWM_PERIOD;
        TIM8->CCR3 = PWM_PERIOD;
        TIM8->CCR2 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[1].HallTab[5])
    {
//        TIM8->CCER = 0x309c;//TIM8->CCER = 0x3abe;//BA
//        TIM8->CCR2 = PWM_Duty;
//        TIM8->CCR3 = PWM_Duty;
        if( PWM_Duty< PWM_Complementary_Switch_M2)
        {
            TIM8->CCER =  BtoA;
        }
        else
        {
            TIM8->CCER = B_complementary_to_A;
        }
        TIM8->CCR2 = PWM_PERIOD;
        TIM8->CCR1 = PWM_PERIOD;
        TIM8->CCR3 = PWM_PERIOD;
        TIM8->CCR2 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[1].HallTab[0])
    {
//        TIM8->CCER = 0x390c;//  TIM8->CCER = 0x3bae;//CA
//        TIM8->CCR2 = PWM_Duty;
//        TIM8->CCR3 = PWM_Duty;
        if( PWM_Duty< PWM_Complementary_Switch_M2)
        {
            TIM8->CCER = CtoA;
        }
        else
        {
            TIM8->CCER = C_complementary_to_A;
        }
        TIM8->CCR2 = PWM_PERIOD;
        TIM8->CCR1 = PWM_PERIOD;
        TIM8->CCR3 = PWM_PERIOD;
        TIM8->CCR3 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[1].HallTab[1])
    {
//        TIM8->CCER = 0x39c0;//TIM8->CCER = 0x3bea;//CB
//        TIM8->CCR1 = PWM_Duty;
//        TIM8->CCR3 = PWM_Duty;
        if( PWM_Duty< PWM_Complementary_Switch_M2 )
        {
            TIM8->CCER = CtoB ;
        }
        else
        {
            TIM8->CCER = C_complementary_to_B;
        }
        TIM8->CCR2 = PWM_PERIOD;
        TIM8->CCR1 = PWM_PERIOD;
        TIM8->CCR3 = PWM_PERIOD;
        TIM8->CCR3 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[1].HallTab[2])
    {
//        TIM8->CCER = 0x30c9;//TIM8->CCER = 0x3aeb;//AB
//        TIM8->CCR3 = PWM_Duty;
//        TIM8->CCR1 = PWM_Duty;
        if( PWM_Duty< PWM_Complementary_Switch_M2)
        {
            TIM8->CCER = AtoB  ;
        }
        else
        {
            TIM8->CCER = A_complementary_to_B;
        }
        TIM8->CCR2 = PWM_PERIOD;
        TIM8->CCR1 = PWM_PERIOD;
        TIM8->CCR3 = PWM_PERIOD;
        TIM8->CCR1 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[1].HallTab[3])
    {
//        TIM8->CCER = 0x3c09;//TIM8->CCER = 0x3eab;//AC
        if( PWM_Duty< PWM_Complementary_Switch_M2)
        {
            TIM8->CCER = AtoC  ;
        }
        else
        {
            TIM8->CCER = A_complementary_to_C;
        }
        TIM8->CCR2 = PWM_PERIOD;
        TIM8->CCR1 = PWM_PERIOD;
        TIM8->CCR3 = PWM_PERIOD;
        TIM8->CCR1 = PWM_Duty;
    }
    else if(bHallState == 7)
    {
        MC_SetFault(HALL6_SENSOR_ERR);
    }
}



void  OneFilter(u16 NewValue,u16 OldValue,u8 FilterFactor )
{
    u16 TempValue1;
    s32 TempValue2;

    TempValue1 = OldValue;
    TempValue2 = (s32)((TempValue1 - NewValue )>>FilterFactor);
    NewValue = (u16)(TempValue2 + NewValue);

//   CurrentVar.AdBuf=data;
//		ValueTemp= CurrentVar.AdBuf;
//		AdTemp=(s32)((ValueTemp-(s32)CurrentVar.AVBuf)>>3);
//	 CurrentVar.AVBuf=(u32)(AdTemp+(s32)CurrentVar.AVBuf);
}

/*------------------------------------------------
Function:霍尔学习换向
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void HALLSTUDY_PhaseChange0(u8 bHallState)
{
    switch (bHallState)//HPWMLpwm
    {
    case 0:

        MC_SetFault(HALL5_SENSOR_ERR);

        break;

    case 6:
        TIM1->CCER = BtoCA;//  TIM1->CCER = 0x1bae;//B->CA

        break;

    case 4:
        TIM1->CCER = CBtoA;//TIM1->CCER = 0x1bea;//CB->A

        break;

    case 5:
        TIM1->CCER = CtoAB;//TIM1->CCER = 0x1aeb;//C->AB

        break;

    case 1:
        TIM1->CCER = ACtoB;//TIM1->CCER = 0x1eab;//AC->B

        break;

    case 3:
        TIM1->CCER = AtoBC;//TIM1->CCER = 0x1eba;//A->BC

        break;

    case 2:
        TIM1->CCER = BAtoC;//TIM1->CCER = 0x1abe;//BA->C

        break;

    case 7:
        MC_SetFault(HALL5_SENSOR_ERR);

        break;


    default:
        break;
    }

}


/*------------------------------------------------
Function:霍尔学习换向
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void HALLSTUDY_PhaseChange1(u8 bHallState)						/* 霍尔学习换向 */
{
    switch (bHallState)//HPWMLpwm
    {
    case 0:

        MC_SetFault(HALL6_SENSOR_ERR);

        break;

    case 6:
        TIM8->CCER = BtoCA;//  TIM8->CCER = 0x3bae;//B->CA

        break;

    case 4:
        TIM8->CCER = CBtoA;//TIM8->CCER = 0x3bea;//CB->A
        break;

    case 5:
        TIM8->CCER = CtoAB;//TIM8->CCER = 0x3aeb;//C->AB

        break;

    case 1:
        TIM8->CCER = ACtoB;//TIM8->CCER = 0x3eab;//AC->B

        break;

    case 3:
        TIM8->CCER = AtoBC;//TIM8->CCER = 0x3eba;//A->BC

        break;

    case 2:
        TIM8->CCER = BAtoC;//TIM8->CCER = 0x3abe;//BA->C

        break;

    case 7:
        MC_SetFault(HALL6_SENSOR_ERR);

        break;


    default:
        break;
    }

}

s16 GetMotorSpeed(u8 Mortor_NUM)         //获取电机实际速度
{
    if(MotorControl[Mortor_NUM].Direction == 1 || MotorControl[Mortor_NUM].Direction == -1)
    {
        MotorControl[Mortor_NUM].Speed_Real = 1000000 *60/(MotorControl[Mortor_NUM].Hall.HALL_CaptureValueDelta * MotorControl[Mortor_NUM].Pole_Paires)*MotorControl[Mortor_NUM].Direction;
        if(HALL_OVF_Counter -MotorControl[Mortor_NUM].Hall.PreHALL_OVF_Counter > 2) MotorControl[Mortor_NUM].Speed_Real = 0 ;
    }
    return MotorControl[Mortor_NUM].Speed_Real;
}


s16 Ramp_Speed(u8 Mortor_NUM)     //速度改变
{
    if(MotorControl[Mortor_NUM].Speed_Set > MotorControl[Mortor_NUM].Speed_Ref)
    {
        MotorControl[Mortor_NUM].Speed_Ref += MotorControl[Mortor_NUM].Acceleration;
        if(MotorControl[Mortor_NUM].Speed_Set < MotorControl[Mortor_NUM].Speed_Ref)
        {
            MotorControl[Mortor_NUM].Speed_Ref = MotorControl[Mortor_NUM].Speed_Set;
        }

    }
    else if(MotorControl[Mortor_NUM].Speed_Set< MotorControl[Mortor_NUM].Speed_Ref)
    {
        MotorControl[Mortor_NUM].Speed_Ref -= MotorControl[Mortor_NUM].Deceleration;
        if(MotorControl[Mortor_NUM].Speed_Set > MotorControl[Mortor_NUM].Speed_Ref)
        {
            MotorControl[Mortor_NUM].Speed_Ref = MotorControl[Mortor_NUM].Speed_Set;
        }

    }



//	if(MotorControl[Mortor_NUM].Speed_Set>0&&PID_Speed_InitStruct[Mortor_NUM].wIntegral < 0)
//	{
//	 PID_Speed_InitStruct[Mortor_NUM].wIntegral =0 ;
//	}
//	else if(MotorControl[Mortor_NUM].Speed_Set<0&&PID_Speed_InitStruct[Mortor_NUM].wIntegral >0)
//	{
//	 PID_Speed_InitStruct[Mortor_NUM].wIntegral =0 ;
//	}

}


s16 Ramp_PPWM(u8 Mortor_NUM)   //改变PWM
{
    if(MotorControl[Mortor_NUM].PWM_DutySet > PWM_PERIOD)
    {
        MotorControl[Mortor_NUM].PWM_DutySet = PWM_PERIOD;
    }
    else if(MotorControl[Mortor_NUM].PWM_DutySet < -PWM_PERIOD)
    {
        MotorControl[Mortor_NUM].PWM_DutySet = -PWM_PERIOD;
    }
    if(MotorControl[Mortor_NUM].PWM_DutySet > MotorControl[Mortor_NUM].PWM_Duty)
    {

        MotorControl[Mortor_NUM].PWM_Duty += MotorControl[Mortor_NUM].Acceleration;
        if(MotorControl[Mortor_NUM].PWM_Duty < 1000)
        {
            MotorControl[Mortor_NUM].PWM_Duty = 1000;
        }
        if(MotorControl[Mortor_NUM].PWM_DutySet < MotorControl[Mortor_NUM].PWM_Duty)
        {
            MotorControl[Mortor_NUM].PWM_Duty = MotorControl[Mortor_NUM].PWM_DutySet;
        }
    }
    else if(MotorControl[Mortor_NUM].PWM_DutySet< MotorControl[Mortor_NUM].PWM_Duty)
    {
        MotorControl[Mortor_NUM].PWM_Duty -= MotorControl[Mortor_NUM].Deceleration;
        if(MotorControl[Mortor_NUM].PWM_Duty > -1000)
        {
            MotorControl[Mortor_NUM].PWM_Duty = -1000;
        }
        if(MotorControl[Mortor_NUM].PWM_DutySet > MotorControl[Mortor_NUM].PWM_Duty)
        {
            MotorControl[Mortor_NUM].PWM_Duty = MotorControl[Mortor_NUM].PWM_DutySet;
        }
    }
}

u8 HALL_GetPhase2(void)  //得到6号电机的hall状态
{
    int32_t tmp = 0;
//  tmp |= HAL_GPIO_ReadPin(HU2_GPIO_Port, HU2_Pin);//U(A)
//  tmp <<= 1;
//  tmp |= HAL_GPIO_ReadPin(HV2_GPIO_Port, HV2_Pin);//V(B)
//  tmp <<= 1;
//  tmp |= HAL_GPIO_ReadPin(HW2_GPIO_Port, HW2_Pin);//W(C)
//  return (u8)(tmp & 0x0007); // ????????

    tmp |= HAL_GPIO_ReadPin(HALL_W2_GPIO_Port, HALL_W2_Pin);//U(A)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(HALL_V2_GPIO_Port, HALL_V2_Pin);//V(B)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(HALL_U2_GPIO_Port, HALL_U2_Pin);//W(C)
    return (u8)(tmp & 0x0007); // ????????
}

void HallStudyHandle0(void)  //5号电机霍尔学习
{
    switch(HALL_Study[0].CommuntionState)
    {
    case 0:
        if(HALL_Study[0].StudySectorCnt < 12)
        {
            u8 HALL_Studytemp = 0;
            if(++HALL_Study[0].StudySectorCnt2 > HALL_Study[0].StudySectorCnt3 )
            {

                HALL_Study[0].StudySectorCnt2 =0;

                HALL_Study[0].HallSector ++;
                if(HALL_Study[0].HallSector>6)
                {
                    HALL_Study[0].HallSector = 1;
                }
                if(HALL_Study[0].HallSector == 1) HALL_Studytemp = 6;
                if(HALL_Study[0].HallSector == 2) HALL_Studytemp = 4;
                if(HALL_Study[0].HallSector == 3) HALL_Studytemp = 5;
                if(HALL_Study[0].HallSector == 4) HALL_Studytemp = 1;
                if(HALL_Study[0].HallSector == 5) HALL_Studytemp = 3;
                if(HALL_Study[0].HallSector == 6) HALL_Studytemp = 2;
                HALL_Study[0].StudySectorCnt++;

                TIM1->CCR1 = HALL_Study[0].HallCommPWM;
                TIM1->CCR2 = HALL_Study[0].HallCommPWM;
                TIM1->CCR3 = HALL_Study[0].HallCommPWM;
                HALLSTUDY_PhaseChange0(HALL_Studytemp);
            }

        }
        else
        {
            HALL_Study[0].CommuntionState = 1;
        }
        break;

    case 1:
        if(HALL_Study[0].HallTab[0]!=HALL_Study[0].HallTab[1]&&
                HALL_Study[0].HallTab[1]!=HALL_Study[0].HallTab[2]&&
                HALL_Study[0].HallTab[2]!=HALL_Study[0].HallTab[3]&&
                HALL_Study[0].HallTab[3]!=HALL_Study[0].HallTab[4]&&
                HALL_Study[0].HallTab[4]!=HALL_Study[0].HallTab[5])
        {
            MC_ClearFault(HALL5_SENSOR_ERR);
            HALL_Study[0].CommuntionState = 3;
        }
        else
        {
            HALL_Study[0].CommuntionState = 2;
        }
        break;

    case 2:
        /*HALL?í?ó*/
//        MC_SetFault(HALL5_SENSOR_ERR);
        HALL_Study[0].CommuntionState = 3;

        break;
    case 3:


        break;
    }

}

void HallStudyHandle1(void)  //6号电机Hall 学习
{
    switch(HALL_Study[1].CommuntionState)
    {
    case 0:
        if(HALL_Study[1].StudySectorCnt < 12)
        {
            u8 HALL_Studytemp = 0;
            if(++HALL_Study[1].StudySectorCnt2 > HALL_Study[1].StudySectorCnt3 )
            {

                HALL_Study[1].StudySectorCnt2 =0;

                HALL_Study[1].HallSector ++;
                if(HALL_Study[1].HallSector>6)
                {
                    HALL_Study[1].HallSector = 1;
                }
                if(HALL_Study[1].HallSector == 1) HALL_Studytemp = 6;
                if(HALL_Study[1].HallSector == 2) HALL_Studytemp = 4;
                if(HALL_Study[1].HallSector == 3) HALL_Studytemp = 5;
                if(HALL_Study[1].HallSector == 4) HALL_Studytemp = 1;
                if(HALL_Study[1].HallSector == 5) HALL_Studytemp = 3;
                if(HALL_Study[1].HallSector == 6) HALL_Studytemp = 2;
                HALL_Study[1].StudySectorCnt++;

                HALLSTUDY_PhaseChange1(HALL_Studytemp);
                TIM8->CCR1 = HALL_Study[1].HallCommPWM;
                TIM8->CCR2 = HALL_Study[1].HallCommPWM;
                TIM8->CCR3 = HALL_Study[1].HallCommPWM;

            }

        }
        else
        {
            HALL_Study[1].CommuntionState = 1;
        }
        break;

    case 1:
        if(HALL_Study[1].HallTab[0]!=HALL_Study[1].HallTab[1]&&
                HALL_Study[1].HallTab[1]!=HALL_Study[1].HallTab[2]&&
                HALL_Study[1].HallTab[2]!=HALL_Study[1].HallTab[3]&&
                HALL_Study[1].HallTab[3]!=HALL_Study[1].HallTab[4]&&
                HALL_Study[1].HallTab[4]!=HALL_Study[1].HallTab[5])
        {
            MC_ClearFault(HALL6_SENSOR_ERR);
            HALL_Study[1].CommuntionState = 3;
        }
        else
        {
            HALL_Study[1].CommuntionState = 2;
        }
        break;

    case 2:
        /*HALL?í?ó*/
        MC_SetFault(HALL6_SENSOR_ERR);
        HALL_Study[1].CommuntionState = 3;

        break;
    case 3:

        break;

    }

}


/**
  * 函数功能: 设置电机转动方向
  * 输入函数: Dir,电机转动方向
  * 返 回 值: 无
  * 说    明: 无
  */

void SetMotorDir(int16_t Dir)    //设置转速，没用到过
{ 
    if(Dir)
    {
        HAL_TIM_PWM_Start(&htimx_BDCMOTOR,TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(&htimx_BDCMOTOR,TIM_CHANNEL_1);         // ????????
    }
    else
    {
        HAL_TIM_PWM_Stop(&htimx_BDCMOTOR,TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(&htimx_BDCMOTOR,TIM_CHANNEL_1);         // ????????
    }
}
void Suction_motor_errhandle(u8 Duty_ratio)                        //S线风机不同的占空比对应的状态
{
	static u8 errcnt;
	if(Duty_ratio-PWM_OFFSET<VOLTAGE_ERROR&&Duty_ratio+PWM_OFFSET>VOLTAGE_ERROR)	//17-23
	{
		errcnt++;
		if(errcnt>100)
		{
			MotorControl[8].PWM_Duty = 0;
			MotorControl[8].Motor_Start_Stop = 0;
			MotorControl[8].Fault_Flag = 1;
			MC_SetFault(FAN_ERROR);
		}
	}
	else if(Duty_ratio-PWM_OFFSET<M8_OVER_CURRENT&&Duty_ratio+PWM_OFFSET>M8_OVER_CURRENT)	//37-43
	{
		errcnt++;
		if(errcnt>100)
		{
			MotorControl[8].PWM_Duty = 0;
			MotorControl[8].Motor_Start_Stop = 0;
			MotorControl[8].Fault_Flag = 1;
			MC_SetFault(FAN_ERROR);
		}
	}
	else if(Duty_ratio-PWM_OFFSET<OVER_SPEED_M8&&Duty_ratio+PWM_OFFSET>OVER_SPEED_M8)		//57-63
	{
		errcnt++;
		if(errcnt>100)
		{
			MotorControl[8].PWM_Duty = 0;
			MotorControl[8].Motor_Start_Stop = 0;
			MotorControl[8].Fault_Flag = 1;
			MC_SetFault(FAN_ERROR);
		}
	}
	else if(Duty_ratio-PWM_OFFSET<OVER_TMP_M8&&Duty_ratio+PWM_OFFSET>OVER_TMP_M8)		//77-83
	{
		errcnt++;
		if(errcnt>100)
		{
			MotorControl[8].PWM_Duty = 0;
			MotorControl[8].Motor_Start_Stop = 0;
			MotorControl[8].Fault_Flag = 1;
			MC_SetFault(FAN_ERROR);
		}
	}
	else if(Duty_ratio-PWM_OFFSET<M8_STUCK&&Duty_ratio+PWM_OFFSET>M8_STUCK)		//97-103
	{
		errcnt++;
		if(errcnt>100)
		{
			MotorControl[8].PWM_Duty = 0;
			MotorControl[8].Motor_Start_Stop = 0;
			MotorControl[8].Fault_Flag = 1;
			MC_SetFault(FAN_ERROR);
		}
	}
	else 
	{
		MotorControl[8].Fault_Flag = 0;
		MC_ClearFault(FAN_ERROR);
		errcnt = 0;
	}
		
}
/******************* (C) COPYRIGHT 2015-2020 ?????????????????? *****END OF FILE****/
