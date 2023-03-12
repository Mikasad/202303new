/**
  ******************************************************************************

  ******************************************************************************

  ******************************************************************************
  */
/* °ü???????? ----------------------------------------------------------------*/
#include "bsp_BDCMotor.h"
#include "main.h"
#include "function.h"
/* ?????à???¨?? --------------------------------------------------------------*/
/* ?????ê?¨?? ----------------------------------------------------------------*/
/* ????±??? ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htimx_BDCMOTOR;
MotorControlParameters_t MotorControl[MOTOR_NUM];
Sensorless_t Sensorless[2];
HALL_Study_t HALL_Study[2];
u8 fan_errcnt;
u16 fan_timer_cnt = 0;
int8_t  Motor_Type=0;         //滚刷电机选择，0：宝龙滚刷电机   1：和泰滚刷电机
int8_t  Push_Type=0;         //推杆电机选择，0：180hall   1：360hall
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



void SetMotorSpeed(uint8_t Motor_NUM,int16_t PWM_Duty)		/* 速度设定 */
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
    case 0:								//推杆0

        if(MotorControl[0].PercentOfPWM > 0)  //正转
        {
            MotorControl[0].Direction = CW;
            if(MotorControl[0].PercentOfPWM > MAX_PERCENT)
            {
                MotorControl[0].PercentOfPWM = MAX_PERCENT;
            }
        }
        else if(MotorControl[0].PercentOfPWM < 0)  //反转
        {
            MotorControl[0].Direction = CCW;
            if(MotorControl[0].PercentOfPWM < -MAX_PERCENT)
            {
                MotorControl[0].PercentOfPWM = -MAX_PERCENT;
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
                }
                else
                {
                    TIM2->CCR3 = 0;
                    TIM2->CCR4 = 0;
                    MotorControl[0].PWM_Duty = 0;

                }
						}
						else
						{
								Ramp_PPWM(0);                  //电机0按照加速度运行
								MOTOR0CW();
								MotorControl[0].Hall.HALL_CaptureValue_Direction_Judgment = 2;
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
                }
                else
                {
                    TIM2->CCR3 = 0;
                    TIM2->CCR4 = 0;
                    MotorControl[0].PWM_Duty = 0;
                }
            }
            else
            {
								MotorControl[0].Hall.HALL_CaptureValue_Direction_Judgment = 1;
                Ramp_PPWM(0);
                MOTOR0CCW();
            }
            break;
        case STOPSTOP:
            MOTOR0STOP();
            break;
        }

        break;

    case 1:									//推杆1

        if(MotorControl[1].PercentOfPWM > 0)  //正转
        {
            MotorControl[1].Direction = CW;
            if(MotorControl[1].PercentOfPWM > MAX_PERCENT)
            {
                MotorControl[1].PercentOfPWM = MAX_PERCENT;
            }
        }
        else if(MotorControl[1].PercentOfPWM < 0)  //反转
        {
            MotorControl[1].Direction = CCW;
            if(MotorControl[1].PercentOfPWM < -MAX_PERCENT)
            {
                MotorControl[1].PercentOfPWM = -MAX_PERCENT;
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
                }
                else
                {
                    TIM9->CCR1 = 0;
                    TIM9->CCR2 = 0;
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
                }
                else
                {
                    TIM9->CCR1 = 0;
                    TIM9->CCR2 = 0;
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



    case 3:						//单向有刷1
				TIM10->PSC = 10000/MotorControl[3].Frequency-1;
//	HAL_GPIO_WritePin(GPIOB,Pump2_H_Pin, GPIO_PIN_RESET); //新增913
//	HAL_GPIO_WritePin(GPIOB,Pump2_L_Pin, GPIO_PIN_SET); //新增913

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
            TIM10->CCR1  = 2*PWM_PERIOD + PWM_PeriodOFFSET;
        }
				TIM4->CCR1 = PWM_PERIOD+PWM_PeriodOFFSET;

        break;

    case 4:						//单向有刷2
				TIM11->PSC = 10000/MotorControl[4].Frequency-1;
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
            TIM11->CCR1  = 2*PWM_PERIOD +PWM_PeriodOFFSET;
        }
				TIM4->CCR2 = PWM_PERIOD+PWM_PeriodOFFSET;
        break;

    case 5:						//无刷1


//        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
//		    HAL_TIM_OC_Stop_IT(&htim5,TIM_CHANNEL_1);

        if(MotorControl[5].Speed_Ref < 0)
        {
//            MotorControl[5].Hall.HallState_CCW = 0x07 ^ MotorControl[5].Hall.HallState;
					
            BLDC1_PhaseChange( MotorControl[5].Hall.HallState_CCW,PWM_Duty);
        }
        else if(MotorControl[5].Speed_Ref > 0)
        {

//            MotorControl[5].Hall.HallState_CW = MotorControl[5].Hall.HallState;
            BLDC1_PhaseChange( MotorControl[5].Hall.HallState_CW,PWM_Duty);
        }
        else
        {
            PWM_Duty =0;
            MotorControl[5].Hall.HallState_CW = MotorControl[5].Hall.HallState;
						MotorControl[5].Hall.HallState_CCW = 0x07 ^ MotorControl[5].Hall.HallState;
            BLDC1_PhaseChange( MotorControl[5].Hall.HallState_CCW,PWM_Duty);
			      TIM1->CCR1 = 0;
						TIM1->CCR2 = 0;
						TIM1->CCR3 = 0;
						PID_Speed_InitStruct[0].wIntegral = 0;
						PID_Current_InitStructure[0].wIntegral = 0;
        }
//        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
//				HAL_TIM_OC_Start_IT(&htim5,TIM_CHANNEL_1);
				
        break;

    case 6:						//无刷2

        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
        if(MotorControl[6].Speed_Ref < 0)
        {
//            MotorControl[6].Hall.HallState_CCW = 0x07 ^ MotorControl[6].Hall.HallState;
            BLDC2_PhaseChange( MotorControl[6].Hall.HallState_CCW,PWM_Duty);
        }
        else if(MotorControl[6].Speed_Ref > 0)
        {
//            MotorControl[6].Hall.HallState_CW = MotorControl[6].Hall.HallState;
            BLDC2_PhaseChange( MotorControl[6].Hall.HallState_CW,PWM_Duty);
        }
        else
        {
            PWM_Duty =0;
            MotorControl[6].Hall.HallState_CW = MotorControl[6].Hall.HallState;
						MotorControl[6].Hall.HallState_CCW = 0x07 ^ MotorControl[6].Hall.HallState;
            BLDC2_PhaseChange( MotorControl[6].Hall.HallState_CW,PWM_Duty);
						PID_Speed_InitStruct[1].wIntegral = 0;
						PID_Current_InitStructure[1].wIntegral = 0;
        }
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
//				Ramp_Speed(6);
        break;

    case 7:    												//边刷
//        HAL_GPIO_WritePin(GPIOD, SideBrush_BRAKE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,SideBrush_BRAKE_Pin,GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOD,SideBrush_F_R_Pin|SideBrush_BRAKE_Pin,GPIO_PIN_SET);
//                Ramp_PPWM(7);
//        if(PWM_Duty > 0)
//        {
//            TIM8->CCR4 =   MotorControl[7].PWM_Duty;
//        }
//        else if(PWM_Duty < 0)
//        {
//            TIM8->CCR4 = - MotorControl[7].PWM_Duty; ;
//        }
//				else 
//        {
//            TIM8->CCR4 = 0;
//        }
//		HAL_GPIO_WritePin(GPIOD,SideBrush_F_R_Pin|SideBrush_BRAKE_Pin,GPIO_PIN_SET);
	    Ramp_PPWM(7);
        if(PWM_Duty > 0)
        {
            TIM8->CCR4 = PWM_PERIOD - MotorControl[7].PWM_Duty;
        }
        else if(PWM_Duty < 0)
        {
            TIM8->CCR4 = PWM_PERIOD + MotorControl[7].PWM_Duty;
        }
				else 
        {
            TIM8->CCR4 = PWM_PERIOD;
        }
				break;



    case 8:    		  //风机
				FAN_ON;
        Ramp_PPWM(8);
        if(PWM_Duty > 0)
        {
			TIM4->CCR4 = MotorControl[8].PWM_Duty;
        }
        else if(PWM_Duty < 0)
        {
			TIM4->CCR4 = -MotorControl[8].PWM_Duty;
        }
		else 
        {
			TIM4->CCR4 = 0;
        }
        break;


    default:
        break;
    }

}



void SetMotorStop(uint8_t Motor_NUM)
{


    switch ( Motor_NUM )
    {
    case 0:

        MOTOR0STOP();
        MotorControl[0].SpeedLimit = 1;
        break;

    case 1:

        MOTOR1STOP();
        break;


    case 3:
        MotorControl[3].PWM_Duty = 0;
        TIM4->CCR1 = PWM_PERIOD+PWM_PeriodOFFSET ;
				TIM10->CCR1 = 2*PWM_PERIOD+PWM_PeriodOFFSET ;

        break;

    case 4: 
        MotorControl[4].PWM_Duty = 0;
        TIM4->CCR2 = PWM_PERIOD+PWM_PeriodOFFSET ;
				TIM11->CCR1 = 2*PWM_PERIOD+PWM_PeriodOFFSET ;


        break;

    case 5:
		if(MotorControl[5].BrulessMode==1)
		{
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;
			MotorControl[5].PWM_Duty = 0;
			MotorControl[5].Speed_Ref = 0;
			PID_Speed_InitStruct[0].wIntegral = 0;
			PID_Current_InitStructure[0].wIntegral = 0;
			MotorControl[5].Hall.HallState_CW = MotorControl[5].Hall.HallState;
			MotorControl[5].Hall.HallState_CCW = 0x07 ^ MotorControl[5].Hall.HallState;
			TIM1->CCER = CLOSE_ALL_TUBE;
		}
        break;

    case 6:
        TIM8->CCR1 = 0;
        TIM8->CCR2 = 0;
        TIM8->CCR3 = 0;
        MotorControl[6].PWM_Duty = 0;
        MotorControl[6].Speed_Ref = 0;
        PID_Speed_InitStruct[1].wIntegral = 0;
				PID_Current_InitStructure[1].wIntegral = 0;
        if(MotorControl[6].Speed_Real ==0 )
        {
//            TIM8->CCER = 0x0555;
					TIM8->CCER = OPEN_ALL_DOWN_TUBE;
        }
				else 
//					TIM8->CCER = 0x0000;
				TIM8->CCER = CLOSE_ALL_TUBE;
        break;

    case 7:
       HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9, GPIO_PIN_RESET);
       HAL_GPIO_WritePin(GPIOD,SideBrush_BRAKE_Pin,GPIO_PIN_RESET);

           TIM8->CCR4= PWM_PERIOD+PWM_PeriodOFFSET;    //913修改
        MotorControl[7].PWM_Duty = 1;
        MotorControl[7].Speed_Real = 0;

        break;


    case 8:
				FAN_OFF;
				fan_errcnt = 0;
				fan_timer_cnt = 0;
        TIM4->CCR4= 0;
//				TIM4->PSC = 0;
        MotorControl[8].PWM_Duty = 0;
        MotorControl[8].Speed_Real = 0;
        break;


    default:
        break;
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
u16 PWM_Complementary_Switch_M1 = 8400,PWM_Complementary_Switch_M2 = 8400;
void BLDC1_PhaseChange(u8 bHallState, s16 PWM_Duty)		/* 换向 */
{
//	  __disable_irq();
    if(PWM_Duty < 0) PWM_Duty = -PWM_Duty;
    if(bHallState == 0)
    {
        MC_SetFault(HALL5_SENSOR_ERR);
    }
    else if(bHallState == HALL_Study[0].HallTab[4])
    {
				if( PWM_Duty< PWM_Complementary_Switch_M1 )
				{
					TIM1->CCER = BtoC;
				}
				else
				{
					TIM1->CCER = B_complementary_to_C;
				}
        TIM1->CCR1 = PWM_Duty;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR2 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[0].HallTab[5])
    {
				if( PWM_Duty< PWM_Complementary_Switch_M1 )
				{
					TIM1->CCER = BtoA;
				}
				else
				{
					TIM1->CCER = B_complementary_to_A;
				}
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_Duty;
        TIM1->CCR2 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[0].HallTab[0])
    {
				if( PWM_Duty< PWM_Complementary_Switch_M1 )
				{
					TIM1->CCER = CtoA;
				}
				else
				{
					TIM1->CCER = C_complementary_to_A;
				}

        TIM1->CCR2 = PWM_Duty;
        TIM1->CCR1 = PWM_PERIOD;
        TIM1->CCR3 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[0].HallTab[1])
    {
				if( PWM_Duty< PWM_Complementary_Switch_M1 )
				{
					TIM1->CCER = CtoB;
				}
				else
				{
					TIM1->CCER = C_complementary_to_B;
				}
        TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR1 = PWM_Duty;
        TIM1->CCR3 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[0].HallTab[2])
    {
				if( PWM_Duty< PWM_Complementary_Switch_M1 )
				{
					TIM1->CCER = AtoB;
				}
				else
				{
					TIM1->CCER = A_complementary_to_B;
				}
        TIM1->CCR2 = PWM_PERIOD;
        TIM1->CCR3 = PWM_Duty;
        TIM1->CCR1 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[0].HallTab[3])
    {
				if( PWM_Duty< PWM_Complementary_Switch_M1 )
				{
					TIM1->CCER = AtoC;
				}
				else
				{
					TIM1->CCER = A_complementary_to_C;
				}
        TIM1->CCR2 = PWM_Duty;
        TIM1->CCR3 = PWM_PERIOD;
        TIM1->CCR1 = PWM_Duty;
    }
    else if(bHallState == 7)
    {
        MC_SetFault(HALL5_SENSOR_ERR);
    }
		
		if(MotorControl[5].Speed_Ref == 0)
		{
			  TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
        TIM1->CCR1 = 0;
		}
//  __enable_irq();
}

void BLDC2_PhaseChange(u8 bHallState, s16 PWM_Duty)		/* 换向 */
{
    if(PWM_Duty < 0) PWM_Duty = -PWM_Duty;

    if(bHallState == 0)
    {
        MC_SetFault(HALL6_SENSOR_ERR);
    }
    else if(bHallState == HALL_Study[1].HallTab[4])
    {
				if( PWM_Duty< PWM_Complementary_Switch_M2 )
				{
					TIM8->CCER = BtoC;
				}
				else
				{
					TIM8->CCER = B_complementary_to_C;
				}
        TIM8->CCR1 = PWM_Duty;
        TIM8->CCR3 = PWM_PERIOD;
        TIM8->CCR2 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[1].HallTab[5])
    {
				if( PWM_Duty< PWM_Complementary_Switch_M2 )
				{
					TIM8->CCER = BtoA;
				}
				else
				{
					TIM8->CCER = B_complementary_to_A;
				}
        TIM8->CCR1 = PWM_PERIOD;
        TIM8->CCR3 = PWM_Duty;
        TIM8->CCR2 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[1].HallTab[0])
    {
				if( PWM_Duty< PWM_Complementary_Switch_M2 )
				{
					TIM8->CCER = CtoA;
				}
				else
				{
					TIM8->CCER = C_complementary_to_A;
				}
        TIM8->CCR2 = PWM_Duty;
        TIM8->CCR1 = PWM_PERIOD;
        TIM8->CCR3 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[1].HallTab[1])
    {
				if( PWM_Duty< PWM_Complementary_Switch_M2 )
				{
					TIM8->CCER = CtoB;
				}
				else
				{
					TIM8->CCER = C_complementary_to_B;
				}
        TIM8->CCR2 = PWM_PERIOD;
        TIM8->CCR1 = PWM_Duty;
        TIM8->CCR3 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[1].HallTab[2])
    {
				if( PWM_Duty< PWM_Complementary_Switch_M2 )
				{
					TIM8->CCER = AtoB;
				}
				else
				{
					TIM8->CCER = A_complementary_to_B;
				}
        TIM8->CCR2 = PWM_PERIOD;
        TIM8->CCR3 = PWM_Duty;
        TIM8->CCR1 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[1].HallTab[3])
    {
				if( PWM_Duty< PWM_Complementary_Switch_M2 )
				{
					TIM8->CCER = AtoC;
				}
				else
				{
					TIM8->CCER = A_complementary_to_C;
				}
        TIM8->CCR2 = PWM_Duty;
        TIM8->CCR3 = PWM_PERIOD;
        TIM8->CCR1 = PWM_Duty;
    }
    else if(bHallState == 7)
    {
        MC_SetFault(HALL6_SENSOR_ERR);
    }
}
void HALLSTUDY_PhaseChange0(u8 bHallState)						/* 霍尔学习换向 */
{
    switch (bHallState)//HPWMLpwm
    {
    case 0:

        MC_SetFault(HALL5_SENSOR_ERR);

        break;

    case 6:
        TIM1->CCER = BtoCA;//  TIM1->CCER = 0x1bae;//CA

        break;

    case 4:
        TIM1->CCER = CBtoA;//TIM1->CCER = 0x1bea;//CB

        break;

    case 5:
        TIM1->CCER = CtoAB;//TIM1->CCER = 0x1aeb;//AB

        break;

    case 1:
        TIM1->CCER = ACtoB;//TIM1->CCER = 0x1eab;//AC

        break;

    case 3:
        TIM1->CCER = AtoBC;//TIM1->CCER = 0x1eba;//BC

        break;

    case 2:
        TIM1->CCER = BAtoC;//TIM1->CCER = 0x1abe;//BA

        break;

    case 7:
        MC_SetFault(HALL5_SENSOR_ERR);

        break;


    default:
        break;
    }

}
void HALLSTUDY_PhaseChange1(u8 bHallState)						/* 霍尔学习换向 */
{
    switch (bHallState)//HPWMLpwm
    {
    case 0:

        MC_SetFault(HALL6_SENSOR_ERR);

        break;

    case 6:
        TIM8->CCER = BtoCA;//  TIM8->CCER = 0x3bae;//CA

        break;

    case 4:
        TIM8->CCER = CBtoA;//TIM8->CCER = 0x3bea;//CB

        break;

    case 5:
        TIM8->CCER = CtoAB;//TIM8->CCER = 0x3aeb;//AB

        break;

    case 1:
        TIM8->CCER = ACtoB;//TIM8->CCER = 0x3eab;//AC

        break;

    case 3:
        TIM8->CCER = AtoBC;//TIM8->CCER = 0x3eba;//BC

        break;

    case 2:
        TIM8->CCER = BAtoC;//TIM8->CCER = 0x3abe;//BA

        break;

    case 7:
        MC_SetFault(HALL6_SENSOR_ERR);

        break;


    default:
        break;
    }

}

s16 GetMotorSpeed(u8 Mortor_NUM)											/* 速度获取 */
{
    if(MotorControl[Mortor_NUM].Direction == 1 || MotorControl[Mortor_NUM].Direction == -1)
    {
        MotorControl[Mortor_NUM].Speed_Real = 1000000 *60/(MotorControl[Mortor_NUM].Hall.HALL_CaptureValueDelta * MotorControl[Mortor_NUM].Pole_Paires)*MotorControl[Mortor_NUM].Direction;
        if(HALL_OVF_Counter -MotorControl[Mortor_NUM].Hall.PreHALL_OVF_Counter > 2) MotorControl[Mortor_NUM].Speed_Real = 0 ;
    }
//		MotorControl[Mortor_NUM].PercentOfSpeed = MotorControl[Mortor_NUM].Speed_Real/BLDC_SPEED_COEEFFOCIENT;
    return MotorControl[Mortor_NUM].Speed_Real;
}


s16 Ramp_Speed(u8 Mortor_NUM)						/* 速度调整 */
{

		MotorControl[Mortor_NUM].Speed_Set = -MotorControl[Mortor_NUM].PercentOfPWM*BLDC_SPEED_COEEFFOCIENT;
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


s16 Ramp_PPWM(u8 Mortor_NUM)			/* PWM调整 */
{
    if(MotorControl[Mortor_NUM].PercentOfPWM > PWM_PECENT)
    {
        MotorControl[Mortor_NUM].PercentOfPWM = PWM_PECENT;
    }
    else if(MotorControl[Mortor_NUM].PercentOfPWM < -PWM_PECENT)
    {
        MotorControl[Mortor_NUM].PercentOfPWM = -PWM_PECENT;
    }
    if(MotorControl[Mortor_NUM].PercentOfPWM*PWM_COEFFICIENT > MotorControl[Mortor_NUM].PWM_Duty)
    {

        MotorControl[Mortor_NUM].PWM_Duty += MotorControl[Mortor_NUM].Acceleration;
        if(MotorControl[Mortor_NUM].PWM_Duty < 1000)
        {
            MotorControl[Mortor_NUM].PWM_Duty = 1000;
        }
        if(MotorControl[Mortor_NUM].PercentOfPWM*PWM_COEFFICIENT < MotorControl[Mortor_NUM].PWM_Duty)
        {
            MotorControl[Mortor_NUM].PWM_Duty = MotorControl[Mortor_NUM].PercentOfPWM*PWM_COEFFICIENT;
        }
    }
    else if(MotorControl[Mortor_NUM].PercentOfPWM*PWM_COEFFICIENT< MotorControl[Mortor_NUM].PWM_Duty)
    {
        MotorControl[Mortor_NUM].PWM_Duty -= MotorControl[Mortor_NUM].Deceleration;
        if(MotorControl[Mortor_NUM].PWM_Duty > -1000)
        {
            MotorControl[Mortor_NUM].PWM_Duty = -1000;
        }
        if(MotorControl[Mortor_NUM].PercentOfPWM*PWM_COEFFICIENT > MotorControl[Mortor_NUM].PWM_Duty)
        {
            MotorControl[Mortor_NUM].PWM_Duty = MotorControl[Mortor_NUM].PercentOfPWM*PWM_COEFFICIENT;
        }
    }
}
u8 HALL_GetPhase1(void)		/* 读取霍尔信号 */
{
    u8 tmp = 0;

    tmp |= HAL_GPIO_ReadPin(Hall_W1_GPIO_Port, Hall_W1_Pin);//U(A)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(Hall_V1_GPIO_Port, Hall_V1_Pin);//V(B)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(Hall_U1_GPIO_Port, Hall_U1_Pin);//W(C)
    return (u8)(tmp & 0x7); 
} 
u8 HALL_GetPhase2(void)		/* 读取霍尔信号 */
{
    int32_t tmp = 0;


    tmp |= HAL_GPIO_ReadPin(Hall_W2_GPIO_Port, Hall_W2_Pin);//U(A)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(Hall_V2_GPIO_Port, Hall_V2_Pin);//V(B)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(Hall_U2_GPIO_Port, Hall_U2_Pin);//W(C)
    return (u8)(tmp & 0x0007); // ????????
}

void HallStudyHandle0(void)		
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
        MC_SetFault(HALL5_SENSOR_ERR);
        HALL_Study[0].CommuntionState = 3;

        break;
    case 3:


        break;
    }

}

void HallStudyHandle1(void)
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

void SetMotorDir(int16_t Dir)
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

int16_t PWM_DutyControl(u8 addpwm_cnt,int16_t PWM_Dutytemp)
{
	if(addpwm_cnt==PWM_ADDCOUNT)
	{
		PWM_Dutytemp  = PWM_Dutytemp*1.2f; 
	}
	else 
	{
		PWM_Dutytemp = PWM_Dutytemp*0.97f; 
	}
	if(PWM_Dutytemp>8200) PWM_Dutytemp = 8200;
	return PWM_Dutytemp;
}
void Suction_motor_errhandle(u8 Duty_ratio)			//风机报警处理
{
	u8 err_cnt_max = 50;
	if(Duty_ratio-PWM_OFFSET<VOLTAGE_ERROR&&Duty_ratio+PWM_OFFSET>VOLTAGE_ERROR)	//17-23
	{
		fan_errcnt++;
		if(VoltVar.err_vol_record == 0&&VoltVar.BUS<VOLT_160V)
		{
			VoltVar.err_vol_record = VoltVar.BUS;
		}
		if(fan_errcnt>err_cnt_max)
		{
			MotorControl[8].PWM_Duty = 0;
			MotorControl[8].Motor_Start_Stop = DISABLE;
			MotorControl[8].Fault_Flag = 1;
			MC_SetFault1(MOTOR8_VOLTAGE_ERROR);
			MC_SetFault(FAN_ERROR);
		}
	}
	else if(Duty_ratio-PWM_OFFSET<M8_OVER_CURRENT&&Duty_ratio+PWM_OFFSET>M8_OVER_CURRENT)	//37-43
	{
		fan_errcnt++;
		if(fan_errcnt>err_cnt_max)
		{
			MotorControl[8].PWM_Duty = 0;
			MotorControl[8].Motor_Start_Stop = DISABLE;
			MotorControl[8].Fault_Flag = 1;
			MC_SetFault1(MOTOR8_OVER_CURRENT);
			MC_SetFault(FAN_ERROR);
		}
	}
	else if(Duty_ratio-PWM_OFFSET<OVER_SPEED_M8&&Duty_ratio+PWM_OFFSET>OVER_SPEED_M8)		//57-63
	{
		fan_errcnt++;
		if(fan_errcnt>err_cnt_max)
		{
			MotorControl[8].PWM_Duty = 0;
			MotorControl[8].Motor_Start_Stop = DISABLE;
			MotorControl[8].Fault_Flag = 1;
			MC_SetFault1(MOTOR8_OVER_SPEED);
			MC_SetFault(FAN_ERROR);
		}
	}
	else if(Duty_ratio-PWM_OFFSET<OVER_TMP_M8&&Duty_ratio+PWM_OFFSET>OVER_TMP_M8)		//77-83
	{
		fan_errcnt++;
		if(fan_errcnt>err_cnt_max)
		{
			MotorControl[8].PWM_Duty = 0;
			MotorControl[8].Motor_Start_Stop = DISABLE;
			MotorControl[8].Fault_Flag = 1;
			MC_SetFault1(MOTOR8_OVER_TEMP);
			MC_SetFault(FAN_ERROR);
		}
	}
	else if(Duty_ratio-PWM_OFFSET<M8_STUCK&&Duty_ratio+PWM_OFFSET>M8_STUCK)		//97-103
	{
		fan_errcnt++;
		if(fan_errcnt>err_cnt_max)
		{
			MotorControl[8].PWM_Duty = 0;
			MotorControl[8].Motor_Start_Stop = DISABLE;
			MotorControl[8].Fault_Flag = 1;
			MC_SetFault1(MOTOR8_STUCK);
			MC_SetFault(FAN_ERROR);
		}
	}
	else 
	{
		if(fan_errcnt>0)
		fan_errcnt--;
		else 
		{
			MotorControl[8].Fault_Flag = 0;
			MC_ClearFault(FAN_ERROR);
			fan_errcnt = 0;
		}
		
	}
		
}
u8 Hall_circle_cnt =  0,Hall_circle_cnt1 = 0;	//一圈hall计数
u8 PWM_Correction = 0;												//pwm补偿开关
u8 angleflag = 0;															//超前角开关
void BLDC_Hall_Handle(u8 motor_num)
{
	u32 timecount = 0;
	if(motor_num==5)
	{
		timecount = __HAL_TIM_GetCounter(&htim5);
		if(angleflag)
		{
			if(MotorControl[5].Speed_Ref > 0&&MotorControl[5].Speed_Real > 900)
			{
				if(PWM_Correction==1)
				{
					MotorControl[5].Hall.Hall_Changecnt = PWM_ADDCOUNT;
				}
				if(Hall_circle_cnt==0)	
				{
					MotorControl[5].Hall.HallState_CW = 3;
					Hall_circle_cnt=1;
				}
				else if(Hall_circle_cnt==1)	
				{
					MotorControl[5].Hall.HallState_CW = 2;
					Hall_circle_cnt=2;
				}
				else if(Hall_circle_cnt==2)	
				{
					MotorControl[5].Hall.HallState_CW = 6;
					Hall_circle_cnt=3;
				}
				else if(Hall_circle_cnt==3)	
				{
					MotorControl[5].Hall.HallState_CW = 4;
					Hall_circle_cnt=4;
				}
				else if(Hall_circle_cnt==4)	
				{
					MotorControl[5].Hall.HallState_CW = 5;
					Hall_circle_cnt=5;
				}
				else if(Hall_circle_cnt ==5)
				{
					MotorControl[5].Hall.HallState_CW = 1;
					Hall_circle_cnt=0;
				}
//				__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,timecount+MotorControl[5].Hall.chgperiod);	
				BLDC1_PhaseChange( MotorControl[5].Hall.HallState_CW,MotorControl[5].PWM_Duty);
			}
			else if(MotorControl[5].Speed_Ref <= 0)
			{
				if(MotorControl[5].Speed_Real<-1300)
				{
					if(PWM_Correction==1)
					{
						MotorControl[5].Hall.Hall_Changecnt = PWM_ADDCOUNT;
					}
					if(Hall_circle_cnt==0)	
					{
						if(MotorControl[5].Hall.HallState_CCW == 2||MotorControl[5].Hall.HallState_CCW == 6)
						{
							MotorControl[5].Hall.HallState_CCW = 2;
						}
						Hall_circle_cnt=1;
					}
					else if(Hall_circle_cnt==1)	
					{
						if(MotorControl[5].Hall.HallState_CCW == 2||MotorControl[5].Hall.HallState_CCW == 3)
						{
							MotorControl[5].Hall.HallState_CCW = 3;
						}
//						MotorControl[5].Hall.HallState_CCW = 3;
						Hall_circle_cnt=2;
					}
					else if(Hall_circle_cnt==2)	
					{
						if(MotorControl[5].Hall.HallState_CCW == 1||MotorControl[5].Hall.HallState_CCW == 3)
						{
							MotorControl[5].Hall.HallState_CCW = 1;
						}
//						MotorControl[5].Hall.HallState_CCW = 1;
						Hall_circle_cnt=3;
					}
					else if(Hall_circle_cnt==3)	
					{
						if(MotorControl[5].Hall.HallState_CCW == 1||MotorControl[5].Hall.HallState_CCW == 5)
						{
							MotorControl[5].Hall.HallState_CCW = 5;
						}
//						MotorControl[5].Hall.HallState_CCW = 5;
						Hall_circle_cnt=4;
					}
					else if(Hall_circle_cnt==4)	
					{
						if(MotorControl[5].Hall.HallState_CCW == 5||MotorControl[5].Hall.HallState_CCW == 4)
						{
							MotorControl[5].Hall.HallState_CCW = 4;
						}
						
//						MotorControl[5].Hall.HallState_CCW = 4;
						Hall_circle_cnt=5;
					}
					else if(Hall_circle_cnt ==5)
					{
						if(MotorControl[5].Hall.HallState_CCW == 4||MotorControl[5].Hall.HallState_CCW == 6)
						{
							MotorControl[5].Hall.HallState_CCW = 6;
						}
//						MotorControl[5].Hall.HallState_CCW = 6;
						Hall_circle_cnt=0;
					}
					BLDC1_PhaseChange( MotorControl[5].Hall.HallState_CCW,MotorControl[5].PWM_Duty);
				}
				
			}
			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,timecount+MotorControl[5].Hall.chgperiod);
		}
	}
	else if(motor_num==6)
	{
		timecount = __HAL_TIM_GetCounter(&htim12);
		if(angleflag)
		{
			if(MotorControl[6].Speed_Ref > 0&&MotorControl[6].Speed_Real > 900)
			{
				MotorControl[6].Hall.Hall_Changecnt = PWM_ADDCOUNT;
				if(Hall_circle_cnt1==0)	
				{
					MotorControl[6].Hall.HallState_CW = 3;
					Hall_circle_cnt1=1;
				}
				else if(Hall_circle_cnt1==1)	
				{
					MotorControl[6].Hall.HallState_CW = 2;
					Hall_circle_cnt1=2;
				}
				else if(Hall_circle_cnt1==2)	
				{
					MotorControl[6].Hall.HallState_CW = 6;
					Hall_circle_cnt1=3;
				}
				else if(Hall_circle_cnt1==3)	
				{
					MotorControl[6].Hall.HallState_CW = 4;
					Hall_circle_cnt1=4;
				}
				else if(Hall_circle_cnt1==4)	
				{
					MotorControl[6].Hall.HallState_CW = 5;
					Hall_circle_cnt1=5;
				}
				else if(Hall_circle_cnt1 ==5)
				{
					MotorControl[6].Hall.HallState_CW = 1;
					Hall_circle_cnt1=0;
				}
//				__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,timecount+MotorControl[6].Hall.chgperiod);	
				BLDC2_PhaseChange( MotorControl[6].Hall.HallState_CW,MotorControl[6].PWM_Duty);
			}
			else if(MotorControl[6].Speed_Ref < 0&&MotorControl[6].Speed_Real < 900)
			{
				if(Hall_circle_cnt1==0)	
				{
					MotorControl[6].Hall.HallState_CCW = 2;
					Hall_circle_cnt1=1;
				}
				else if(Hall_circle_cnt1==1)	
				{
					MotorControl[6].Hall.HallState_CCW = 3;
					Hall_circle_cnt1=2;
				}
				else if(Hall_circle_cnt1==2)	
				{
					MotorControl[6].Hall.HallState_CCW = 1;
					Hall_circle_cnt1=3;
				}
				else if(Hall_circle_cnt1==3)	
				{
					MotorControl[6].Hall.HallState_CCW = 5;
					Hall_circle_cnt1=4;
				}
				else if(Hall_circle_cnt1==4)	
				{
					MotorControl[6].Hall.HallState_CCW = 4;
					Hall_circle_cnt=5;
				}
				else if(Hall_circle_cnt1 ==5)
				{
					MotorControl[6].Hall.HallState_CCW = 6;
					Hall_circle_cnt1=0;
				}
//				__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,timecount+MotorControl[6].Hall.chgperiod);	
				BLDC2_PhaseChange( MotorControl[6].Hall.HallState_CCW,MotorControl[6].PWM_Duty);
			}
			__HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,timecount+MotorControl[6].Hall.chgperiod);	
		}
	}
}
/******************* (C) COPYRIGHT 2015-2020 ?????????????????? *****END OF FILE****/
