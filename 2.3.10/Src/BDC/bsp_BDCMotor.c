#include "bsp_BDCMotor.h"
#include "main.h"
TIM_HandleTypeDef htimx_BDCMOTOR;
MotorControlParameters_t MotorControl[MOTOR_NUM];
HALL_Study_t HALL_Study[2];
int8_t  Motor_Type=0;         //滚刷电机选择，0：滚刷电机   1：盘刷电机（无刷）  2：盘刷电机（有刷）
int8_t  FilterMotor_Type=0; 
/*************************
*Function Name 		:SetMotorSpeed
*Description   		:设置电机速度
* Input           : Motor_NUM  PWM_Duty
* Output          : None
* Return          : None		
*************************/
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
                    TIM2->CCR3 = 0;
                    TIM2->CCR4 = 0;
                    HAL_GPIO_WritePin(GPIOD,PWM_AL2_Pin,GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOD,PWM_BL2_Pin,GPIO_PIN_SET);
                    MotorControl[1].PWM_Duty = 0;
                }
            }
            else
            {
                MotorControl[1].Hall.HALL_CaptureValue_Direction_Judgment = 2;
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
                    TIM2->CCR3 = 0;
                    TIM2->CCR4 = 0;
                    HAL_GPIO_WritePin(GPIOD,PWM_AL2_Pin,GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOD,PWM_BL2_Pin,GPIO_PIN_RESET);
                    MotorControl[1].PWM_Duty = 0;
                }
            }
            else
            {
                MotorControl[1].Hall.HALL_CaptureValue_Direction_Judgment = 1;
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
        if(Motor_Type<2)
		{
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
				if(Motor_Type==0)
				{
				  MotorControl[5].Hall.HallState_CW = MotorControl[5].Hall.HallState;
				  BLDC1_PhaseChange( MotorControl[5].Hall.HallState_CW,PWM_Duty);
				}
				if(Motor_Type==1)          //盘刷电机转速相反
				{
				 MotorControl[5].Hall.HallState_CCW = 0x07 ^ MotorControl[5].Hall.HallState;   //11.7
				  BLDC1_PhaseChange( MotorControl[5].Hall.HallState_CCW,PWM_Duty);
				}
				PID_Speed_InitStruct[0].wIntegral = 0;
				PID_Current_InitStructure[0].wIntegral = 0;
			}
			HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
		}
		else 
		{
			 Ramp_PPWM(5);                  //电机5按照加速度运行
			 BLDC1_PhaseChange(1,MotorControl[5].PWM_Duty);   // 
		}

        break;

    case 6:
          if(Motor_Type<2)
		{
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
		}
		else 
		{
			 Ramp_PPWM(6);                  //电机6按照加速度运行
			 BLDC2_PhaseChange(1,MotorControl[6].PWM_Duty);   // 
		}

        break;

    case 7:    																																						  															//边刷

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
        break;



    case 8:    		  //风机

        Ramp_PPWM(8);
        if(PWM_Duty > 0)
        {
            TIM3->CCR4 = PWM_PERIOD - MotorControl[8].PWM_Duty;
        }
        else if(PWM_Duty < 0)
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

    case 10:  //电磁阀取消
        MotorControl[10].PWM_DutySet = 8400;
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

        break;
    case 11: //电动阀
        MotorControl[11].PWM_DutySet = 8400;
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
        break;
    case 12:         //单向有刷
        MotorControl[12].PWM_DutySet = 8400;
        Ramp_PPWM(12);
        if(MotorControl[12].PWM_Duty < 0)
        {
            MotorControl[12].PWM_Duty = -MotorControl[12].PWM_Duty;
        }
        TIM8->CCR4 =  MotorControl[12].PWM_Duty;
        break;

    case 13: 	        //蠕动泵
        Ramp_PPWM(13);
        if(MotorControl[13].PWM_Duty < 0)
        {
            MotorControl[13].PWM_Duty = -MotorControl[13].PWM_Duty;
        }
        TIM1->CCR4 = MotorControl[13].PWM_Duty;
        break;


    default:
        break;
    }

}
/*************************
*Function Name 		:SetMotorStop
*Description   		:设置电机停止
* Input           : Motor_NUM  
* Output          : None
* Return          : None		
*************************/
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

    case 2:
        MOTOR2STOP();
        break;

    case 3:
        MotorControl[3].PWM_Duty = 0;
        TIM10->CCR1 = 2*(PWM_PERIOD+PWM_PeriodOFFSET) ;

        break;

    case 4: 
        MotorControl[4].PWM_Duty = 0;
        TIM11->CCR1 = 2*(PWM_PERIOD+PWM_PeriodOFFSET) ;


        break;

    case 5:
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
        MotorControl[5].PWM_Duty = 0;
        MotorControl[5].Speed_Ref = 0;
        PID_Speed_InitStruct[0].wIntegral = 0;
        PID_Current_InitStructure[0].wIntegral = 0;
		    TIM1->CCER = CLOSE_ALL_TUBE;
        break;

    case 6:
        TIM8->CCR1 = 0;
        TIM8->CCR2 = 0;
        TIM8->CCR3 = 0;
        MotorControl[6].PWM_Duty = 0;
        MotorControl[6].Speed_Ref = 0;
        PID_Speed_InitStruct[1].wIntegral = 0;
        PID_Current_InitStructure[1].wIntegral = 0;
        TIM8->CCER = CLOSE_ALL_TUBE;
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

/*************************
*Function Name 		:BLDC1_PhaseChange
*Description   		:换向函数
* Input           : bHallState     PWM_Duty
* Output          : None
* Return          : None		
*************************/
void BLDC1_PhaseChange(u8 bHallState, s16 PWM_Duty)
{
	if(Motor_Type<2)
	{
		 __disable_irq();
		if(PWM_Duty < 0) PWM_Duty = -PWM_Duty;
		if(bHallState == 0)
		{
//			MC_SetFault(HALL5_SENSOR_ERR);
		}
		else if(bHallState == HALL_Study[0].HallTab[4])
		{
			if( PWM_Duty< PWM_Complementary_Switch_M1 ) //重载控制方式改为互补模式，降低mos开关损耗，该测试未50机器加装散热片方案
			{
				TIM1->CCER = BtoC ;       //B->C
			}
			else
			{
				TIM1->CCER = B_complementary_to_C;       //B(complementary)->C
			}
			TIM1->CCR1 = PWM_Duty;
			TIM1->CCR3 = PWM_PERIOD;
			TIM1->CCR2 = PWM_Duty;
		}
		else if(bHallState == HALL_Study[0].HallTab[5])
		{
			if( PWM_Duty< PWM_Complementary_Switch_M1 )
			{
				TIM1->CCER = BtoA; //B->A
			}
			else
			{
				TIM1->CCER = B_complementary_to_A; //  //B(complementary)->A
			}
			TIM1->CCR1 = PWM_PERIOD;
			TIM1->CCR3 = PWM_Duty;
			TIM1->CCR2 = PWM_Duty;
		}
		else if(bHallState == HALL_Study[0].HallTab[0])
		{
			if( PWM_Duty< PWM_Complementary_Switch_M1 )
			{
				TIM1->CCER = CtoA; //C->A
			}
			else
			{
				TIM1->CCER = C_complementary_to_A;  //C(complementary)->A
			}
			TIM1->CCR2 = PWM_Duty;
			TIM1->CCR1 = PWM_PERIOD;
			TIM1->CCR3 = PWM_Duty;
		}
		else if(bHallState == HALL_Study[0].HallTab[1])
		{
			if( PWM_Duty< PWM_Complementary_Switch_M1 )
			{
				TIM1->CCER = CtoB    ; //C->B
			}
			else
			{
				TIM1->CCER = C_complementary_to_B; //C(complementary)->B
			}
			TIM1->CCR2 = PWM_PERIOD;
			TIM1->CCR1 = PWM_Duty;
			TIM1->CCR3 = PWM_Duty;
		}
		else if(bHallState == HALL_Study[0].HallTab[2])
		{
			if( PWM_Duty< PWM_Complementary_Switch_M1 )
			{
				TIM1->CCER = AtoB;  //A->B
			}
			else
			{
				TIM1->CCER = A_complementary_to_B ;  //A(complementary)->B
			}
			 TIM1->CCR2 = PWM_PERIOD;
			TIM1->CCR3 = PWM_Duty;
			TIM1->CCR1 = PWM_Duty;
		}
		else if(bHallState == HALL_Study[0].HallTab[3])
		{
			if( PWM_Duty< PWM_Complementary_Switch_M1 )
			{
				TIM1->CCER =  AtoC; //A->C
			}
			else
			{
				TIM1->CCER = A_complementary_to_C; //A(complementary)->C
			}
			TIM1->CCR2 = PWM_Duty;
			TIM1->CCR3 = PWM_PERIOD;
			TIM1->CCR1 = PWM_Duty;
		}
		else if(bHallState == 7)
		{
//			MC_SetFault(HALL5_SENSOR_ERR);
		}
		 __enable_irq();
	}
	else
	{
		TIM1->CCER = BtoA;  //B->A  逆时针，1月3号进行修改
		TIM1->CCR2 = PWM_PERIOD;
		TIM1->CCR1 = PWM_PERIOD;
		TIM1->CCR3 = PWM_PERIOD;
		TIM1->CCR2 = PWM_Duty;
	}
}
/*************************
*Function Name 		:BLDC2_PhaseChange
*Description   		:换向函数
* Input           : bHallState     PWM_Duty
* Output          : None
* Return          : None		
*************************/
void BLDC2_PhaseChange(u8 bHallState, s16 PWM_Duty)
{
	if(Motor_Type<2)
	{
		 __disable_irq();
		if(PWM_Duty < 0) PWM_Duty = -PWM_Duty;
		if(bHallState == 0)
		{
//			MC_SetFault(HALL6_SENSOR_ERR);
		}
		else if(bHallState == HALL_Study[1].HallTab[4])
		{
			if( PWM_Duty< PWM_Complementary_Switch_M2)
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
			if( PWM_Duty< PWM_Complementary_Switch_M2)
			{
				TIM8->CCER =  BtoA;
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
			if( PWM_Duty< PWM_Complementary_Switch_M2)
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
				TIM8->CCER = CtoB ;
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
			if( PWM_Duty< PWM_Complementary_Switch_M2)
			{
				TIM8->CCER = AtoB  ;
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
			if( PWM_Duty< PWM_Complementary_Switch_M2)
			{
				TIM8->CCER = AtoC  ;
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
//			MC_SetFault(HALL6_SENSOR_ERR);
		}
		 __enable_irq();
	}
	else
	{
		TIM8->CCER = AtoB;  //A->B
		TIM8->CCR2 = PWM_PERIOD;
		TIM8->CCR3 = PWM_PERIOD;
		TIM8->CCR1 = PWM_Duty;
	}
}
/*************************
*Function Name 		:HALLSTUDY_PhaseChange0
*Description   		:霍尔学习函数
* Input           : bHallState     
* Output          : None
* Return          : None		
*************************/
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
/*************************
*Function Name 		:HALLSTUDY_PhaseChange1
*Description   		:霍尔学习函数
* Input           : bHallState     
* Output          : None
* Return          : None		
*************************/
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
/*************************
*Function Name 		:GetMotorSpeed
*Description   		:获取转速
* Input           : Mortor_NUM     
* Output          : None
* Return          : None		
*************************/
s16 GetMotorSpeed(u8 Mortor_NUM)											/* 速度获取 */
{
        MotorControl[Mortor_NUM].Speed_Real = 1000000 *60/(MotorControl[Mortor_NUM].Hall.HALL_CaptureValueDelta * MotorControl[Mortor_NUM].Pole_Paires)*MotorControl[Mortor_NUM].Direction;
        if(HALL_OVF_Counter -MotorControl[Mortor_NUM].Hall.PreHALL_OVF_Counter > 2)
        {
            MotorControl[Mortor_NUM].Speed_Real = 0 ;
        }
    return MotorControl[Mortor_NUM].Speed_Real;
}
/*************************
*Function Name 		:Ramp_Speed
*Description   		:加速减速函数
* Input           : Mortor_NUM     
* Output          : None
* Return          : None		
*************************/
s16 Ramp_Speed(u8 Mortor_NUM)						/* 速度调整 */
{
    if(MotorControl[Mortor_NUM].Speed_Set>0&&MotorControl[Mortor_NUM].Speed_Set<300)
    {
        MotorControl[Mortor_NUM].Speed_Set = 250;
    }
    if(MotorControl[Mortor_NUM].Speed_Set<0&&MotorControl[Mortor_NUM].Speed_Set>-300)
    {
        MotorControl[Mortor_NUM].Speed_Set = -250;
    }
	if(Motor_Type==0)  //滚刷限速1000
	{
		if(MotorControl[Mortor_NUM].Speed_Set>1000)
		{
			MotorControl[Mortor_NUM].Speed_Set = 1000;
		}
		if(MotorControl[Mortor_NUM].Speed_Set<0)
		{
			MotorControl[Mortor_NUM].Speed_Set = 0;    //下位机下发反向转速，滚刷速度为0
		}
	}
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
}
/*------------------------------------------------
Function:PWM斜坡规划
Input   :Mortor_NUM
Output  :No
Explain :No
------------------------------------------------*/
s16 Ramp_PPWM(u8 Mortor_NUM)			/* PWM调整 */
{
    if(MotorControl[Mortor_NUM].PWM_DutySet > PWM_PERIOD)
    {
        MotorControl[Mortor_NUM].PWM_DutySet = PWM_PERIOD;
    }
    else if(MotorControl[Mortor_NUM].PWM_DutySet < -PWM_PERIOD)
    {
        MotorControl[Mortor_NUM].PWM_DutySet = -PWM_PERIOD;
    }
    if(MotorControl[Mortor_NUM].PWM_DutySet == 0)
    {
        MotorControl[Mortor_NUM].PWM_Duty = 0;
    }
    if(MotorControl[Mortor_NUM].PWM_DutySet > MotorControl[Mortor_NUM].PWM_Duty)
    {
        MotorControl[Mortor_NUM].PWM_Duty += MotorControl[Mortor_NUM].Acceleration;
    }
    else if(MotorControl[Mortor_NUM].PWM_DutySet< MotorControl[Mortor_NUM].PWM_Duty)
    {
        MotorControl[Mortor_NUM].PWM_Duty -= MotorControl[Mortor_NUM].Deceleration;
    }
	 if(MotorControl[3].PWM_DutySet > 1000)   //过滤电机限制PWM幅值
    {
        MotorControl[3].PWM_DutySet = 8000;
    }
}
/*************************
*Function Name 		:HALL_GetPhase1
*Description   		:得到hall状态函数
* Input           : Mortor_NUM     
* Output          : None
* Return          : None		
*************************/
u8 HALL_GetPhase2(void)		/* 读取霍尔信号 */
{
    int32_t tmp = 0;
    tmp |= HAL_GPIO_ReadPin(HALL_W2_GPIO_Port, HALL_W2_Pin);//U(A)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(HALL_V2_GPIO_Port, HALL_V2_Pin);//V(B)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(HALL_U2_GPIO_Port, HALL_U2_Pin);//W(C)
    return (u8)(tmp & 0x0007); 
}
/*------------------------------------------------
Function:无刷电机霍尔学习函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
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
                if(HALL_Study[0].HallSector == 1) 
				{	
				    HALL_Studytemp = 6;
				}
                if(HALL_Study[0].HallSector == 2) 
				{	
			    	HALL_Studytemp = 4;
				}
                if(HALL_Study[0].HallSector == 3) 
				{	
				    HALL_Studytemp = 5;
				}
                if(HALL_Study[0].HallSector == 4) 
				{	
				    HALL_Studytemp = 1;
				}
                if(HALL_Study[0].HallSector == 5) 
				{	
				    HALL_Studytemp = 3;
				}
                if(HALL_Study[0].HallSector == 6) 
				{	
				    HALL_Studytemp = 2;
				}
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
/*------------------------------------------------
Function:无刷电机霍尔学习函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
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
                if(HALL_Study[1].HallSector == 1) 
				{
					HALL_Studytemp = 6;
				}
                if(HALL_Study[1].HallSector == 2) 
				{
			    	HALL_Studytemp = 4;
				}
                if(HALL_Study[1].HallSector == 3) 
				{	
			    	HALL_Studytemp = 5;
				}
                if(HALL_Study[1].HallSector == 4) 
				{	
				    HALL_Studytemp = 1;
				}
                if(HALL_Study[1].HallSector == 5) 
				{	
				    HALL_Studytemp = 3;
				}
                if(HALL_Study[1].HallSector == 6) 
				{	
				    HALL_Studytemp = 2;
				}
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
/******************* (C) COPYRIGHT 2015-2020 ?????????????????? *****END OF FILE****/
