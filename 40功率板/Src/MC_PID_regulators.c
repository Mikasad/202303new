/******************** (C) COPYRIGHT TOPBAND ********************
* File Name          : MC_PID_regulators.c
* Author             : Guoxl
* Date First Issued  :
* Description        :
********************************************************************************
* COPYRIGHT TOPBAND
* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/

//#include "stm32f10x_lib.h"
//#include "stm32f10x_MClib.h"
//#include "stm32f10x_type.h"
//#include "MC_Globals.h"

#include "MC_PID_regulators.h"
//#include "bsp_BDCMotor.h"
PID_Struct_t   PID_Speed_InitStruct[2];
volatile s16 hSpeed_Reference;

PID_Struct_t   PID_Current_InitStructure[2];
PID_Struct_t	 PID_PWM;
volatile s16 hCurrent_Reference;

#define PID_SPEED_REFERENCE  (u16)(PID_SPEED_REFERENCE_RPM/1)//6
typedef signed long long s64;

/*******************************************************************************
* Function Name  : PID_Init
* Description    : Initialize PID coefficients for torque, flux and speed loop:
                   Kp_Gain: proportional coeffcient
                   Ki_Gain: integral coeffcient
                   Kd_Gain: differential coeffcient
* Input          : Pointer 1 to Torque PI structure,
                   Pointer 2 to Flux PI structure,
                   Pointer 3 to Speed PI structure
* Output         : None
* Return         : None
*******************************************************************************/
void PID_Init (PID_Struct_t *PID_Speed, PID_Struct_t *PID_Current)		//速度环位置环PID初始化
{


    /**************************************************/
    /************PID Speed Regulator members*************/
    /**************************************************/


    PID_Speed->wIntegral = 0;  // reset integral value

//  hSpeed_Reference = PID_SPEED_REFERENCE;
    PID_Speed->hReference = 0;
    PID_Speed->hKp_Gain    = PID_SPEED_KP_DEFAULT;
    PID_Speed->hKp_Divisor = SP_KPDIV;

    PID_Speed->hKi_Gain = PID_SPEED_KI_DEFAULT;
    PID_Speed->hKi_Divisor = SP_KIDIV;

    PID_Speed->hKd_Gain = PID_SPEED_KD_DEFAULT;
    PID_Speed->hKd_Divisor = SP_KDDIV;
    PID_Speed->wPreviousError = 0;

    PID_Speed->hLower_Limit_Output= -MotorControl[5].Current.MaxValue4;   //Lower Limit for Output limitation
    PID_Speed->hUpper_Limit_Output= MotorControl[5].Current.MaxValue4;   //Upper Limit for Output limitation
    PID_Speed->wLower_Limit_Integral = -MotorControl[5].Current.MaxValue4 * SP_KIDIV;
    PID_Speed->wUpper_Limit_Integral = MotorControl[5].Current.MaxValue4 * SP_KIDIV;
    PID_Speed->wIntegral = 0;
    PID_Speed->Ksat = 1;
    /**************************************************/
    /**********END PID Speed Regulator members*********/
    /**************************************************/

    PID_Current->wIntegral = 0;  // reset integral value

    PID_Current->hReference = 0;
    PID_Current->hKp_Gain    = KP_GAIN_CURRENT_MIN;
    PID_Current->hKp_Divisor = CURRENT_KPDIV;

    PID_Current->hKi_Gain = KI_GAIN_CURRENT;
    PID_Current->hKi_Divisor = CURRENT_KIDIV;

    PID_Current->hKd_Gain = KD_GAIN_CURRENT;
    PID_Current->hKd_Divisor = CURRENT_KDDIV;
    PID_Current->wPreviousError = 0;

    PID_Current->hLower_Limit_Output= -IQMAX;   //Lower Limit for Output limitation
    PID_Current->hUpper_Limit_Output= IQMAX ;   //Upper Limit for Output limitation
    PID_Current->wLower_Limit_Integral = -IQMAX * CURRENT_KIDIV;
    PID_Current->wUpper_Limit_Integral = IQMAX * CURRENT_KIDIV;
    PID_Current->wIntegral = 0;
    PID_Current->Ksat = 1;
}

void PID_PWM_Init(PID_Struct_t *PID_PWM)				/* 推杆PID初始化 */
{
		PID_PWM->wIntegral = 0;  // reset integral value

    PID_PWM->hReference = 0;
    PID_PWM->hKp_Gain    = 2000;
    PID_PWM->hKp_Divisor = 128;

    PID_PWM->hKi_Gain = 4;
    PID_PWM->hKi_Divisor = CURRENT_KIDIV;

    PID_PWM->hKd_Gain = KD_GAIN_CURRENT;
    PID_PWM->hKd_Divisor = CURRENT_KDDIV;
    PID_PWM->wPreviousError = 0;

    PID_PWM->hLower_Limit_Output= -6000;   //Lower Limit for Output limitation
    PID_PWM->hUpper_Limit_Output= 6000 ;   //Upper Limit for Output limitation
    PID_PWM->wLower_Limit_Integral = -6000 * CURRENT_KIDIV;
    PID_PWM->wUpper_Limit_Integral = 6000 * CURRENT_KIDIV;
    PID_PWM->wIntegral = 0;
    PID_PWM->Ksat = 1;
}
s16 PID_Regulator(s16 hReference, s16 hPresentFeedback, PID_Struct_t *PID_Struct)
{
    s32 wError, wProportional_Term,wIntegral_Term, houtput_32;
    s64 dwAux;

    // error computation
    wError= (s32)(hReference - hPresentFeedback);

    PID_Struct->hReference = hReference ;
    PID_Struct->hPresentFeedback = hPresentFeedback ;
    PID_Struct->hErr = wError;
    // Proportional term computation
    wProportional_Term = PID_Struct->hKp_Gain * wError;

    // Integral term computation
    if (PID_Struct->hKi_Gain == 0)
    {
        PID_Struct->wIntegral = 0;
    }
    else
    {
        wIntegral_Term = PID_Struct->hKi_Gain * wError + PID_Struct->Ksat*PID_Struct->SatErr;
        dwAux = PID_Struct->wIntegral + (s64)(wIntegral_Term);

        if (dwAux > PID_Struct->wUpper_Limit_Integral)
        {
            PID_Struct->wIntegral = PID_Struct->wUpper_Limit_Integral;
        }
        else if (dwAux < PID_Struct->wLower_Limit_Integral)
        {
            PID_Struct->wIntegral = PID_Struct->wLower_Limit_Integral;
        }
        else
        {
            PID_Struct->wIntegral = (s32)(dwAux);
        }
    }

    PID_Struct->OutPreSat = (wProportional_Term/PID_Struct->hKp_Divisor+
                             PID_Struct->wIntegral/PID_Struct->hKi_Divisor);

    if (PID_Struct->OutPreSat >= PID_Struct->hUpper_Limit_Output*0.95)  //7980 = 8400*0.95
    {
        houtput_32=(PID_Struct->hUpper_Limit_Output);
    }
    else if (PID_Struct->OutPreSat< PID_Struct->hLower_Limit_Output*0.95)
    {
        houtput_32=(PID_Struct->hLower_Limit_Output);
    }
    else
    {
        houtput_32=PID_Struct->OutPreSat;
    }
    PID_Struct->SatErr=houtput_32-PID_Struct->OutPreSat;
    PID_Struct->hOutput = houtput_32 ;
    return((s16)(houtput_32));
}


/******************** (C) COPYRIGHT TOPBAND ******************END OF FILE****/


