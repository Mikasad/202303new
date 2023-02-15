#include "bsp_BDCMotor.h"
#include "zd_driver.h"
#include "main.h"
TIM_HandleTypeDef htimx_BDCMOTOR;
MotorControlParameters_t MotorControl[MOTOR_NUM];
uint8_t Side_brush_report=0;
uint8_t Side_brush_live7=0;
uint8_t Side_brush_live8=0;

uint8_t Roll6_brush_report=0;
uint8_t Roll_Side_brush_live6=0;
HALL_Study_t HALL_Study[2];

/*------------------------------------------------
Function:速度设定函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void SetMotorSpeed(uint8_t Motor_NUM,int16_t PWM_Duty)		/* 速度设定 */
{
    if(PWM_Duty > PWM_PERIOD) //占空比限制
    {
        PWM_Duty = PWM_PERIOD ;
    }
    else if(PWM_Duty <- PWM_PERIOD) //占空比限制
    {
        PWM_Duty = -PWM_PERIOD ;
    }
    switch ( Motor_NUM )
    {
    case 0: //刷盘升降推杆

        if(MotorControl[0].PWM_DutySet > 0)  //正转
        {
            MotorControl[0].Direction = CW;
            if(MotorControl[0].PWM_DutySet > PERCENT_95_OF_PWM_PERIOD) //占空比限制
            {
                MotorControl[0].PWM_DutySet = PERCENT_95_OF_PWM_PERIOD;
            }
        }
        else if(MotorControl[0].PWM_DutySet < 0)  //反转
        {
            MotorControl[0].Direction = CCW;
            if(MotorControl[0].PWM_DutySet < -PERCENT_95_OF_PWM_PERIOD) //占空比限制
            {
                MotorControl[0].PWM_DutySet = -PERCENT_95_OF_PWM_PERIOD;
            }
        }
        else
        {
            MotorControl[0].Direction = STOPSTOP; //停转
        }
        switch(MotorControl[0].Direction)
        {
        case CW: //正转
            if(MotorControl[0].Direction != MotorControl[0].LastMotorDirection) //上次是否为反转 先停下来在正转
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

        case CCW: //反转
            if(MotorControl[0].Direction != MotorControl[0].LastMotorDirection) //上次是否为正转 先停下来在反转
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
                Ramp_PPWM(0);            //电机0按照加速度运行
                MOTOR0CCW();
            }
            break;
        case STOPSTOP:
            MOTOR0STOP();
            break;
        }

        break;

    case 1: //
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
//                    HAL_GPIO_WritePin(GPIOD,PWM_AL2_Pin,GPIO_PIN_SET);
                }
                else
                {
//                    MOTOR1STOP();
                    TIM2->CCR3 = 0;
                    TIM2->CCR4 = 0;
//                    HAL_GPIO_WritePin(GPIOD,PWM_AL2_Pin,GPIO_PIN_RESET);
//                    HAL_GPIO_WritePin(GPIOD,PWM_BL2_Pin,GPIO_PIN_SET);
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
//                    HAL_GPIO_WritePin(GPIOD,PWM_BL2_Pin,GPIO_PIN_SET);
                }
                else
                {
//                    MOTOR1STOP();
                    TIM2->CCR3 = 0;
                    TIM2->CCR4 = 0;
//                    HAL_GPIO_WritePin(GPIOD,PWM_AL2_Pin,GPIO_PIN_SET);
//                    HAL_GPIO_WritePin(GPIOD,PWM_BL2_Pin,GPIO_PIN_RESET);
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


    case 2:  //水趴升降推杆

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
//                    HAL_GPIO_WritePin(GPIOD,PWM_AL3_Pin,GPIO_PIN_SET); //电已经充好，2ms之后将进入MOTOR2CW，在这先关闭需要打开的上管所对应的下管
                }
                else
                {
//                    MOTOR2STOP();
                    TIM4->CCR1 = 0;
                    TIM4->CCR2 = 0;
//                    HAL_GPIO_WritePin(GPIOD,PWM_AL3_Pin,GPIO_PIN_RESET);
//                    HAL_GPIO_WritePin(GPIOD,PWM_BL3_Pin,GPIO_PIN_SET);
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
//                    HAL_GPIO_WritePin(GPIOD,PWM_BL3_Pin,GPIO_PIN_SET);
                }
                else
                {
//                    MOTOR2STOP();
                    TIM4->CCR1 = 0;
                    TIM4->CCR2 = 0;
//                    HAL_GPIO_WritePin(GPIOD,PWM_AL3_Pin,GPIO_PIN_SET);
 //                   HAL_GPIO_WritePin(GPIOD,PWM_BL3_Pin,GPIO_PIN_RESET);
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

    case 3:  //过滤电机

        Ramp_PPWM(3);
        if(PWM_Duty > 0)
        {
//            TIM10->CCR1 = 2*(PWM_PERIOD - MotorControl[3].PWM_Duty);
					  TIM10->CCR1 = 2*(MotorControl[3].PWM_Duty);
        }
        else if(PWM_Duty<0)
        {
//            TIM10->CCR1  = 2*(PWM_PERIOD + MotorControl[3].PWM_Duty);
					  TIM10->CCR1  = 2*(-MotorControl[3].PWM_Duty);
        }
        else
        {
//            TIM10->CCR1  = 2*(PWM_PERIOD + PWM_PeriodOFFSET);
					TIM10->CCR1  = 0;
        }

        break;

    case 4:  //喷水电机

        Ramp_PPWM(4);
        if(PWM_Duty > 0)
        {
//            TIM11->CCR1 = 2*(PWM_PERIOD - MotorControl[4].PWM_Duty);
					  TIM11->CCR1 = 2*(MotorControl[4].PWM_Duty);
        }
        else if(PWM_Duty < 0)
        {
//            TIM11->CCR1  = 2*(PWM_PERIOD + MotorControl[4].PWM_Duty);
					  TIM11->CCR1  = 2*(-MotorControl[4].PWM_Duty);
        }
        else
        {
//            TIM11->CCR1  = 2*(PWM_PERIOD +PWM_PeriodOFFSET);
					  TIM11->CCR1  = 0;
        }

        break;

    case 5:  //滚刷电机

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

    case 6:  //

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

    case 7:  //右边刷   			 																																			  															//边刷
			if(MotorControl[7].Motor_mode == Control_485) //485控制 最大为8400 485控制最大为330RPM 25倍
      {  
		    if((RS_485_TIME==SB_TIME) || (RXC_states==3))  //超时100ms或者右边刷右边刷接收完成
		    {
 //        BmSetPwm(SB_ID,MotorControl[7].PWM_DutySet,MotorControl[7].Acceleration);	
				 BmSetPwm(SB_ID,MotorControl[7].PWM_DutySet/25,MotorControl[7].Acceleration);	//与端口控制数值对其	
				}
			}
			else if(MotorControl[7].Motor_mode == Control_IO) //端口控制
      {
        Ramp_PPWM(7);
        if(PWM_Duty > 0)
        {
					  SB1_FR(0); //使用这个端口做测试，暂时关闭
            TIM3->CCR3 = PWM_PERIOD - MotorControl[7].PWM_Duty;
        }
        else if(PWM_Duty < 0)
        {
					   SB1_FR(1);
            TIM3->CCR3 = PWM_PERIOD + MotorControl[7].PWM_Duty;
        }
        else
        {
            TIM3->CCR3 = PWM_PERIOD + PWM_PeriodOFFSET;
        }
//       TIM3->CCR3 = 0;
			}
        break;

    case 8:  //左边刷
			if(MotorControl[8].Motor_mode == Control_485) //485控制 最大为8400 485控制最大为330 25倍
      {	
		    if((RS_485_TIME==SB2_TIME) ||  (RXC_states==1)) //超时100ms或者风机接收完成
		    {
//        BmSetPwm(SB2_ID,MotorControl[9].PWM_DutySet,MotorControl[9].Acceleration);	
					BmSetPwm(SB2_ID,MotorControl[8].PWM_DutySet/25,MotorControl[8].Acceleration);//与端口控制数值对其	
				}
			}
			else if(MotorControl[8].Motor_mode == Control_IO) //端口控制
      {
        Ramp_PPWM(8);
        if(PWM_Duty > 0)
        {
					  SB2_FR(0);
            TIM4->CCR3 = PWM_PERIOD - MotorControl[8].PWM_Duty;
        }
        else if(PWM_Duty < 0)
//					else
        {
					  SB2_FR(1);
            TIM4->CCR3  = PWM_PERIOD + MotorControl[8].PWM_Duty;
        }
        else
        {
            TIM4->CCR3  = PWM_PERIOD + PWM_PeriodOFFSET;
        }
			}
        break;

    case 9:    		  //风机
			if(MotorControl[9].Motor_mode == Control_485) //485控制 最大为8400 485控制最大为40 210倍
      {
				if((RS_485_TIME==FAN_TIME) ||  (RXC_states==2))//超时100ms或者左边刷接收完成
		    {
//          B86SetPwm(FAN_ID,MotorControl[9].PWM_DutySet); 
          B86SetPwm(FAN_ID,MotorControl[9].PWM_DutySet/210); //与端口控制数值对其	
				}
			}
			else if(MotorControl[9].Motor_mode == Control_IO) //端口控制
      {
        Ramp_PPWM(9);
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
			}
        break;




    case 10:  //
        MotorControl[10].PWM_DutySet = 8400;
        Ramp_PPWM(10);
        if(PWM_Duty > 0)
        {
            TIM3->CCR1 = MotorControl[10].PWM_Duty;
        }
        else if(PWM_Duty < 0)
        {
            TIM3->CCR1 = -MotorControl[10].PWM_Duty;
        }
        else
        {
            TIM3->CCR1 = 0;
        }
//        TIM3->CCR1 = 0;

        break;
    case 11: //排污电机
        MotorControl[11].PWM_DutySet = 300;  //大了会break
        Ramp_PPWM(11);
        if(PWM_Duty > 0)
        {
            TIM3->CCR2 = MotorControl[11].PWM_Duty;
        }
        else if(PWM_Duty < 0)
        {
            TIM3->CCR2 =-MotorControl[11].PWM_Duty;
        }
        else
        {
            TIM3->CCR2 = 0;
        }
//        TIM3->CCR2 = 0;
        break;
    case 12:         //
        MotorControl[12].PWM_DutySet = 8400;
        Ramp_PPWM(12);
        if(PWM_Duty > 0)
        {
            TIM8->CCR4 = PWM_PERIOD - MotorControl[12].PWM_Duty;
        }
        else if(PWM_Duty < 0)
        {
            TIM8->CCR4 = PWM_PERIOD + MotorControl[12].PWM_Duty;
        }
        else
        {
            TIM8->CCR4 = PWM_PERIOD +PWM_PeriodOFFSET;
        }
				
//        if(MotorControl[12].PWM_Duty < 0)
//        {
//            MotorControl[12].PWM_Duty = -MotorControl[12].PWM_Duty;
//        }
//        TIM8->CCR4 =  MotorControl[12].PWM_Duty;
//        TIM8->CCR4 = 8400;
        break;

    case 13: 	        //蠕动泵
        Ramp_PPWM(13);
        if(PWM_Duty > 0)
        {
            TIM1->CCR4 = PWM_PERIOD - MotorControl[13].PWM_Duty;
        }
        else if(PWM_Duty < 0)
        {
            TIM1->CCR4 = PWM_PERIOD + MotorControl[13].PWM_Duty;
        }
        else
        {
            TIM1->CCR4 = PWM_PERIOD +PWM_PeriodOFFSET;
        }
				
//        if(MotorControl[13].PWM_Duty < 0)
//        {
//            MotorControl[13].PWM_Duty = -MotorControl[13].PWM_Duty;
//        }
//        TIM1->CCR4 = MotorControl[13].PWM_Duty;
        break;


    default:
        break;
    }

}
/*------------------------------------------------
Function:停机函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void SetMotorStop(uint8_t Motor_NUM)
{


    switch ( Motor_NUM )
    {
    case 0://刷盘升降推杆

        MOTOR0STOP();
        MotorControl[0].SpeedLimit = 1;
        break;

    case 1://

        MOTOR1STOP();
        break;

    case 2://水趴升降推杆

        MOTOR2STOP();
        break;

    case 3://过滤电机
        MotorControl[3].PWM_Duty = 0;
//        TIM10->CCR1 = 2*(PWM_PERIOD+PWM_PeriodOFFSET) ;
        TIM10->CCR1 = 0 ;
        break;

    case 4: //喷水电机
        MotorControl[4].PWM_Duty = 0;
//        TIM11->CCR1 = 2*(PWM_PERIOD+PWM_PeriodOFFSET) ;
        TIM11->CCR1 = 0 ;

        break;

    case 6://滚刷电机
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
        MotorControl[6].PWM_Duty = 0;
        MotorControl[6].Speed_Ref = 0;
        PID_Speed_InitStruct[1].wIntegral = 0;
        PID_Current_InitStructure[1].wIntegral = 0;
        if(MotorControl[6].Speed_Real ==0 )
        {
            TIM1->CCER = OPEN_ALL_DOWN_TUBE;
        }
        else
        {
            TIM1->CCER = CLOSE_ALL_TUBE;
        }
        break;

    case 5:
        TIM8->CCR1 = 0;
        TIM8->CCR2 = 0;
        TIM8->CCR3 = 0;
        MotorControl[5].PWM_Duty = 0;
        MotorControl[5].Speed_Ref = 0;
        PID_Speed_InitStruct[0].wIntegral = 0;
        PID_Current_InitStructure[0].wIntegral = 0;
        if(MotorControl[5].Speed_Real ==0 )
        {
            TIM8->CCER = OPEN_ALL_DOWN_TUBE;
        }
        else
        {
            TIM8->CCER = CLOSE_ALL_TUBE;
        }
        break;

    case 7://左边刷
			if(MotorControl[7].Motor_mode == Control_485) //
			{			
		    if((RS_485_TIME==SB_TIME) || (RXC_states==3))  //超时100ms或者右边刷右边刷接收完成
		    {
         BmSetPwm(SB_ID,0,MotorControl[7].Acceleration);	
				}
			}
			if(MotorControl[7].Motor_mode == Control_IO) //
			{		
        TIM3->CCR3= PWM_PERIOD+PWM_PeriodOFFSET;
        MotorControl[7].PWM_Duty = 0;
        MotorControl[7].Speed_Real = 0;
			}

        break;
			
    case 8: //右边刷
			if(MotorControl[8].Motor_mode == Control_485) //
			{			
		    if((RS_485_TIME==SB2_TIME) ||  (RXC_states==1)) //超时100ms或者风机接收完成
		    {
         BmSetPwm(SB2_ID,0,MotorControl[8].Acceleration);		
				}
			}
			if(MotorControl[8].Motor_mode == Control_IO) //
			{
        TIM4->CCR3= PWM_PERIOD+PWM_PeriodOFFSET;
        MotorControl[8].PWM_Duty = 0;
        MotorControl[8].Speed_Real = 0;
			}
        break;

    case 9://风机
			if(MotorControl[9].Motor_mode == Control_485) //
			{
				if((RS_485_TIME==FAN_TIME) ||  (RXC_states==2))//超时100ms或者左边刷接收完成
		    {
         B86SetPwm(FAN_ID,0); 
				}
			}
			if(MotorControl[9].Motor_mode == Control_IO) //
			{
        TIM3->CCR4= PWM_PERIOD+PWM_PeriodOFFSET;
        MotorControl[9].PWM_Duty = 0;
        MotorControl[9].Speed_Real = 0;
			}
        break;


    case 10:
        MotorControl[10].PWM_Duty = 0;
//        TIM3->CCR1= PWM_PERIOD+PWM_PeriodOFFSET;
		    TIM3->CCR1= 0;
        break;
    case 11: //排污电机
        MotorControl[11].PWM_Duty = 0;
//        TIM3->CCR2= PWM_PERIOD+PWM_PeriodOFFSET;
		    TIM3->CCR2= 0;
        break;
    case 12:
        MotorControl[12].PWM_Duty = 0;
//        TIM8->CCR4 = 0;
		    TIM8->CCR4 = PWM_PERIOD+PWM_PeriodOFFSET;
        

        break;


    case 13: //蠕动泵
        MotorControl[13].PWM_Duty = 0;
//      TIM1->CCR4 = 0;
        TIM1->CCR4= PWM_PERIOD+PWM_PeriodOFFSET;
        
        break;

    default:
        break;
    }
}
/*------------------------------------------------
Function:一介滤波
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
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
Function:BLDC2换向
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void BLDC2_PhaseChange(u8 bHallState, s16 PWM_Duty)
{
	  __disable_irq();
    if(PWM_Duty < 0) PWM_Duty = -PWM_Duty;
    if(bHallState == 0)
    {
        MC_SetFault(HALL6_SENSOR_ERR);
    }
    else if(bHallState == HALL_Study[1].HallTab[4])
    {
//        if( PWM_Duty< PWM_Complementary_Switch_M1 ) //正向 重载控制方式改为互补模式，降低mos开关损耗，该测试未50机器加装散热片方案
//        {
//            TIM1->CCER = BtoC ;       //B->C
//        }
//        else
//        {
//            TIM1->CCER = B_complementary_to_C;       //B(complementary)->C
//        }
//        TIM1->CCR1 = PWM_Duty;
//        TIM1->CCR3 = PWM_PERIOD;
//        TIM1->CCR2 = PWM_Duty;
	
        if( PWM_Duty< PWM_Complementary_Switch_M1 )  //反向
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
    else if(bHallState == HALL_Study[1].HallTab[5])
    {
//        if( PWM_Duty< PWM_Complementary_Switch_M1 ) //正向
//        {
//            TIM1->CCER = BtoA; //B->A
//        }
//        else
//        {
//            TIM1->CCER = B_complementary_to_A; //  //B(complementary)->A
//        }
//        TIM1->CCR1 = PWM_PERIOD;
//        TIM1->CCR3 = PWM_Duty;
//        TIM1->CCR2 = PWM_Duty;
	
        if( PWM_Duty< PWM_Complementary_Switch_M1 ) //反向
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
    else if(bHallState == HALL_Study[1].HallTab[0])
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
    else if(bHallState == HALL_Study[1].HallTab[1])
    {
//        if( PWM_Duty< PWM_Complementary_Switch_M1 ) //正向
//        {
//            TIM1->CCER = CtoB    ; //C->B
//        }
//        else
//        {
//            TIM1->CCER = C_complementary_to_B; //C(complementary)->B
//        }
//        TIM1->CCR2 = PWM_PERIOD;
//        TIM1->CCR1 = PWM_Duty;
//        TIM1->CCR3 = PWM_Duty;

        if( PWM_Duty< PWM_Complementary_Switch_M1 ) //反向
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
    else if(bHallState == HALL_Study[1].HallTab[2])
    {
//        if( PWM_Duty< PWM_Complementary_Switch_M1 ) //正向
//        {
//            TIM1->CCER = AtoB;  //A->B
//        }
//        else
//        {
//            TIM1->CCER = A_complementary_to_B ;  //A(complementary)->B
//        }
//        TIM1->CCR2 = PWM_PERIOD;
//        TIM1->CCR3 = PWM_Duty;
//        TIM1->CCR1 = PWM_Duty;
				
        if( PWM_Duty< PWM_Complementary_Switch_M1 ) //反向 重载控制方式改为互补模式，降低mos开关损耗，该测试未50机器加装散热片方案
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
    else if(bHallState == HALL_Study[1].HallTab[3])
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
        MC_SetFault(HALL6_SENSOR_ERR);
    }
		if(MotorControl[6].Speed_Ref == 0)
		{
			  TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
        TIM1->CCR1 = 0;
		}
		__enable_irq();
}
/*------------------------------------------------
Function:BLDC1换向
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void BLDC1_PhaseChange(u8 bHallState, s16 PWM_Duty)
{
	  __disable_irq();
    if(PWM_Duty < 0) PWM_Duty = -PWM_Duty;
    if(bHallState == 0)
    {
        MC_SetFault(HALL5_SENSOR_ERR);
    }
    else if(bHallState == HALL_Study[0].HallTab[4])
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
        TIM8->CCR1 = PWM_Duty;
        TIM8->CCR3 = PWM_PERIOD;
        TIM8->CCR2 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[0].HallTab[5])
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
        TIM8->CCR1 = PWM_PERIOD;
        TIM8->CCR3 = PWM_Duty;
        TIM8->CCR2 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[0].HallTab[0])
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
        TIM8->CCR2 = PWM_Duty;
        TIM8->CCR1 = PWM_PERIOD;
        TIM8->CCR3 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[0].HallTab[1])
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
        TIM8->CCR1 = PWM_Duty;
        TIM8->CCR3 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[0].HallTab[2])
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
        TIM8->CCR3 = PWM_Duty;
        TIM8->CCR1 = PWM_Duty;
    }
    else if(bHallState == HALL_Study[0].HallTab[3])
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
        TIM8->CCR2 = PWM_Duty;
        TIM8->CCR3 = PWM_PERIOD;
        TIM8->CCR1 = PWM_Duty;
    }
    else if(bHallState == 7)
    {
        MC_SetFault(HALL5_SENSOR_ERR);
    }
		__enable_irq();
}
/*------------------------------------------------
Function:霍尔学习换向1
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void HALLSTUDY_PhaseChange1(u8 bHallState)
{
    switch (bHallState)//HPWMLpwm
    {
    case 0:

        MC_SetFault(HALL6_SENSOR_ERR);

        break;

    case 6:
//        TIM1->CCER = BtoCA;//  TIM1->CCER = 0x1bae;//B->CA 正向
         TIM1->CCER = ACtoB;//TIM1->CCER = 0x1eab;//AC->B    反向  
        break;

    case 4:
//        TIM1->CCER = CBtoA;//TIM1->CCER = 0x1bea;//CB->A
         TIM1->CCER = AtoBC;//TIM1->CCER = 0x1eba;//A->BC
        break;

    case 5:
//        TIM1->CCER = CtoAB;//TIM1->CCER = 0x1aeb;//C->AB
        TIM1->CCER = BAtoC;//TIM1->CCER = 0x1abe;//BA->C
        break;

    case 1:
//        TIM1->CCER = ACtoB;//TIM1->CCER = 0x1eab;//AC->B
        TIM1->CCER = BtoCA;//  TIM1->CCER = 0x1bae;//B->CA
        break;

    case 3:
//        TIM1->CCER = AtoBC;//TIM1->CCER = 0x1eba;//A->BC
        TIM1->CCER = CBtoA;//TIM1->CCER = 0x1bea;//CB->A
        break;

    case 2:
//        TIM1->CCER = BAtoC;//TIM1->CCER = 0x1abe;//BA->C
        TIM1->CCER = CtoAB;//TIM1->CCER = 0x1aeb;//C->AB
        break;

    case 7:
        MC_SetFault(HALL6_SENSOR_ERR);

        break;


    default:
        break;
    }

}
/*------------------------------------------------
Function:霍尔学习换向0
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void HALLSTUDY_PhaseChange0(u8 bHallState)						/* 霍尔学习换向 */
{
    switch (bHallState)//HPWMLpwm
    {
    case 0:

        MC_SetFault(HALL5_SENSOR_ERR);

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
        MC_SetFault(HALL5_SENSOR_ERR);

        break;


    default:
        break;
    }

}
/*------------------------------------------------
Function:速度获取函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
/* 速度获取 固定脉冲看时间多少 
一个hall周期记一次时间(us):MotorControl[Mortor_NUM].Hall.HALL_CaptureValueDelta 
时间*极对数为电机跑一圈需要的时间(us):MotorControl[Mortor_NUM].Hall.HALL_CaptureValueDelta *MotorControl[Mortor_NUM].Pole_Paires
1/时间是频率：1000000/(MotorControl[Mortor_NUM].Hall.HALL_CaptureValueDelta *MotorControl[Mortor_NUM].Pole_Paires)= HZ(转/S)
HZ(转/S)*60=rpm(转/分钟)
rpm(转/分钟)*方向=正负 rpm(转/分钟)
*/
s16 GetMotorSpeed(u8 Mortor_NUM)											
{
//    if(MotorControl[Mortor_NUM].Direction == 1 || MotorControl[Mortor_NUM].Direction == -1)
    {
        MotorControl[Mortor_NUM].Speed_Real = 1000000 *60/(MotorControl[Mortor_NUM].Hall.HALL_CaptureValueDelta * MotorControl[Mortor_NUM].Pole_Paires)*MotorControl[Mortor_NUM].Direction;
        if(HALL_OVF_Counter -MotorControl[Mortor_NUM].Hall.PreHALL_OVF_Counter > 2) MotorControl[Mortor_NUM].Speed_Real = 0 ;
    }
    return MotorControl[Mortor_NUM].Speed_Real;
}
/*------------------------------------------------
Function:速度规划斜坡
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
s16 Ramp_Speed(u8 Mortor_NUM)						/* 速度调整 */
{
//		if(MotorControl[Mortor_NUM].Speed_Set*MotorControl[Mortor_NUM].Speed_Ref < 0)
//		{
////			MotorControl[Mortor_NUM].Speed_Ref = 0;
//
////		 MotorControl[Mortor_NUM].Speed_Ref = MotorControl[Mortor_NUM].Speed_Set;
//			if(MotorControl[Mortor_NUM].Speed_Set>0)
//				MotorControl[Mortor_NUM].Speed_Ref = 0;
//			else
//			{
//
//			}
//		}
    if(MotorControl[Mortor_NUM].Speed_Set>0&&MotorControl[Mortor_NUM].Speed_Set<300)
    {
        MotorControl[Mortor_NUM].Speed_Set = 250;
    }
    if(MotorControl[Mortor_NUM].Speed_Set<0&&MotorControl[Mortor_NUM].Speed_Set>-300)
    {
        MotorControl[Mortor_NUM].Speed_Set = -250;
    }
//    if(MotorControl[Mortor_NUM].Speed_Set>1000)
//    {
//        MotorControl[Mortor_NUM].Speed_Set = 1000;
//    }
//    if(MotorControl[Mortor_NUM].Speed_Set<-1000)
//    {
//        MotorControl[Mortor_NUM].Speed_Set = -1000;
//    }

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

}/*------------------------------------------------
Function:PWM斜坡规划
Input   :No
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
		if(Mortor_NUM==2) //推杆2特殊处理，因为使用TMI8870芯片 占空比小于50%无动作
		{
			if(MotorControl[Mortor_NUM].PWM_DutySet > MotorControl[Mortor_NUM].PWM_Duty)
			{
				  if(MotorControl[Mortor_NUM].PWM_Duty<4000)
					{
					 MotorControl[Mortor_NUM].PWM_Duty=4000;
					}				  
					 MotorControl[Mortor_NUM].PWM_Duty += MotorControl[Mortor_NUM].Acceleration;
					if(MotorControl[Mortor_NUM].PWM_Duty> MotorControl[Mortor_NUM].PWM_DutySet)
					{
					 MotorControl[Mortor_NUM].PWM_Duty = MotorControl[Mortor_NUM].PWM_DutySet;
					}
			}
			else if(MotorControl[Mortor_NUM].PWM_DutySet< MotorControl[Mortor_NUM].PWM_Duty)
			{
					if(MotorControl[Mortor_NUM].PWM_Duty>-4000)
					{
					 MotorControl[Mortor_NUM].PWM_Duty=-4000;
					}
					MotorControl[Mortor_NUM].PWM_Duty -= MotorControl[Mortor_NUM].Deceleration;
					if(MotorControl[Mortor_NUM].PWM_Duty< MotorControl[Mortor_NUM].PWM_DutySet)
					{
					 MotorControl[Mortor_NUM].PWM_Duty = MotorControl[Mortor_NUM].PWM_DutySet;
					}
			}		
		}
		else
		{
			if(MotorControl[Mortor_NUM].PWM_DutySet > MotorControl[Mortor_NUM].PWM_Duty)
			{
					MotorControl[Mortor_NUM].PWM_Duty += MotorControl[Mortor_NUM].Acceleration;
			}
			else if(MotorControl[Mortor_NUM].PWM_DutySet< MotorControl[Mortor_NUM].PWM_Duty)
			{
					MotorControl[Mortor_NUM].PWM_Duty -= MotorControl[Mortor_NUM].Deceleration;
			}
	  }
}
/*------------------------------------------------
Function:读取霍尔状态
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
u8 HALL_GetPhase2(void)		/* 读取霍尔信号 */
{
    int32_t tmp = 0;
//  tmp |= HAL_GPIO_ReadPin(HU2_GPIO_Port, HU2_Pin);//U(A)
//  tmp <<= 1;
//  tmp |= HAL_GPIO_ReadPin(HV2_GPIO_Port, HV2_Pin);//V(B)
//  tmp <<= 1;
//  tmp |= HAL_GPIO_ReadPin(HW2_GPIO_Port, HW2_Pin);//W(C)
//  return (u8)(tmp & 0x0007); // ????????

    tmp |= HAL_GPIO_ReadPin(HALL_W2_GPIO_Port, HALL_W2_Pin);//W(C)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(HALL_V2_GPIO_Port, HALL_V2_Pin);//V(B)
    tmp <<= 1;
    tmp |= HAL_GPIO_ReadPin(HALL_U2_GPIO_Port, HALL_U2_Pin);//U(A)
    return (u8)(tmp & 0x0007); // ????????
}
/*------------------------------------------------
Function:无刷电机霍尔学习函数1
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
                if(HALL_Study[1].HallSector == 1) HALL_Studytemp = 6; //6
                if(HALL_Study[1].HallSector == 2) HALL_Studytemp = 2; //4
                if(HALL_Study[1].HallSector == 3) HALL_Studytemp = 3; //5
                if(HALL_Study[1].HallSector == 4) HALL_Studytemp = 1; //1
                if(HALL_Study[1].HallSector == 5) HALL_Studytemp = 5; //3
                if(HALL_Study[1].HallSector == 6) HALL_Studytemp = 4; //2
                HALL_Study[1].StudySectorCnt++;

                TIM1->CCR1 = HALL_Study[1].HallCommPWM;
                TIM1->CCR2 = HALL_Study[1].HallCommPWM;
                TIM1->CCR3 = HALL_Study[1].HallCommPWM;
                HALLSTUDY_PhaseChange1(HALL_Studytemp);
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
/*------------------------------------------------
Function:无刷电机霍尔学习函数0
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
                if(HALL_Study[0].HallSector == 1) HALL_Studytemp = 6;
                if(HALL_Study[0].HallSector == 2) HALL_Studytemp = 4;
                if(HALL_Study[0].HallSector == 3) HALL_Studytemp = 5;
                if(HALL_Study[0].HallSector == 4) HALL_Studytemp = 1;
                if(HALL_Study[0].HallSector == 5) HALL_Studytemp = 3;
                if(HALL_Study[0].HallSector == 6) HALL_Studytemp = 2;
                HALL_Study[0].StudySectorCnt++;

                HALLSTUDY_PhaseChange0(HALL_Studytemp);
                TIM8->CCR1 = HALL_Study[0].HallCommPWM;
                TIM8->CCR2 = HALL_Study[0].HallCommPWM;
                TIM8->CCR3 = HALL_Study[0].HallCommPWM;

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
Function:吸风电机错误处理
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void Suction_motor_errhandle(u8 Duty_ratio)
{
    static u8 errcnt;
	
	 if(MotorControl[9].Motor_mode == Control_IO) //
	 {		
    if(Duty_ratio-PWM_OFFSET<VOLTAGE_ERROR&&Duty_ratio+PWM_OFFSET>VOLTAGE_ERROR)	//17-23
    {
        errcnt++;
        if(errcnt>100)
        {
            MotorControl[9].PWM_Duty = 0;
            MotorControl[9].Motor_Start_Stop = 0;
            MotorControl[9].Fault_Flag = 1;
            MC_SetFault1(MOTOR8_VOLTAGE_ERROR);
            MC_SetFault(FAN_ERROR);
        }
    }
    else if(Duty_ratio-PWM_OFFSET<M8_OVER_CURRENT&&Duty_ratio+PWM_OFFSET>M8_OVER_CURRENT)	//37-43
    {
        errcnt++;
        if(errcnt>100)
        {
            MotorControl[9].PWM_Duty = 0;
            MotorControl[9].Motor_Start_Stop = 0;
            MotorControl[9].Fault_Flag = 1;
            MC_SetFault1(MOTOR8_OVER_CURRENT);
            MC_SetFault(FAN_ERROR);
        }
    }
    else if(Duty_ratio-PWM_OFFSET<OVER_SPEED_M8&&Duty_ratio+PWM_OFFSET>OVER_SPEED_M8)		//57-63
    {
        errcnt++;
        if(errcnt>100)
        {
            MotorControl[9].PWM_Duty = 0;
            MotorControl[9].Motor_Start_Stop = 0;
            MotorControl[9].Fault_Flag = 1;
            MC_SetFault1(MOTOR8_OVER_SPEED);
            MC_SetFault(FAN_ERROR);
        }
    }
    else if(Duty_ratio-PWM_OFFSET<OVER_TMP_M8&&Duty_ratio+PWM_OFFSET>OVER_TMP_M8)		//77-83
    {
        errcnt++;
        if(errcnt>100)
        {
            MotorControl[9].PWM_Duty = 0;
            MotorControl[9].Motor_Start_Stop = 0;
            MotorControl[9].Fault_Flag = 1;
            MC_SetFault1(MOTOR8_OVER_TEMP);
            MC_SetFault(FAN_ERROR);
        }
    }
    else if(Duty_ratio-PWM_OFFSET<M8_STUCK&&Duty_ratio+PWM_OFFSET>M8_STUCK)		//97-103
    {
        errcnt++;
        if(errcnt>100)
        {
            MotorControl[9].PWM_Duty = 0;
            MotorControl[9].Motor_Start_Stop = 0;
            MotorControl[9].Fault_Flag = 1;
            MC_SetFault1(MOTOR8_STUCK);
            MC_SetFault(FAN_ERROR);
        }
    }
    else
    {
        MotorControl[9].Fault_Flag = 0;
        MC_ClearFault(FAN_ERROR);
        errcnt = 0;
    }
	}
	 if(MotorControl[9].Motor_mode == Control_485) //
	 {
			if(MotorControl[9].Bm_fault_code !=0)
			{
					MotorControl[9].PWM_Duty = 0;
					MotorControl[9].Motor_Start_Stop = 0;
					MotorControl[9].Fault_Flag = 1;
					 MC_SetFault(FAN_ERROR);
					if(MotorControl[9].Bm_fault_code==0x3 || MotorControl[9].Bm_fault_code==0x4)//过压 欠压
					{
					MC_SetFault1(MOTOR8_VOLTAGE_ERROR);
					}
					else if(MotorControl[9].Bm_fault_code==0x1)//过流
					{
					MC_SetFault1(MOTOR8_OVER_CURRENT);
					}
					else if(MotorControl[9].Bm_fault_code==0x7)//超速
					{
					MC_SetFault1(MOTOR8_OVER_SPEED);	
					}
					else if(MotorControl[9].Bm_fault_code==0x9)//过温
					{
					MC_SetFault1(MOTOR8_OVER_TEMP);
					}		
					else if(MotorControl[9].Bm_fault_code==0x1)//叶轮卡住
					{
					MC_SetFault1(MOTOR8_STUCK);	
					}							
			}
			else
			{
					MC_ClearFault(FAN_ERROR);
			}			 
		 
	 }
}
/******************* (C) COPYRIGHT 2015-2020 ?????????????????? *****END OF FILE****/
