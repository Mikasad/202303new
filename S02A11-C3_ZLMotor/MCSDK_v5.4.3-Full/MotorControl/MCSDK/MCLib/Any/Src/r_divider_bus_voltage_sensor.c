/**
  ******************************************************************************
  * @file    r_divider_bus_voltage_sensor.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the  features
  *          of the Resistor Divider Bus Voltage Sensor component of the Motor
  *          Control SDK:
  *
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

/* Includes ------------------------------------------------------------------*/
#include "r_divider_bus_voltage_sensor.h"
#include "regular_conversion_manager.h"


/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup BusVoltageSensor
  * @{
  */

/** @defgroup RDividerBusVoltageSensor Resistor Divider Bus Voltage Sensor
  * @brief Resistor Divider Bus Voltage Sensor implementation
  *
  * @todo Document the Resistor Divider Bus Voltage Sensor "module".
  *
  * @{
  */

/**
  * @brief  It initializes bus voltage conversion (ADC, ADC channel, conversion time.
    It must be called only after PWMC_Init.
  * @param  pHandle related RDivider_Handle_t
  * @retval none
  */
__weak void RVBS_Init( RDivider_Handle_t * pHandle )
{
    /* Need to be register with RegularConvManager */
    pHandle->convHandle = RCM_RegisterRegConv(&pHandle->VbusRegConv);
    /* Check */
    RVBS_Clear( pHandle );
}


/**
  * @brief  It clears bus voltage FW variable containing average bus voltage
  *         value
  * @param  pHandle related RDivider_Handle_t
  * @retval none
  */
__weak void RVBS_Clear( RDivider_Handle_t * pHandle )
{
    uint16_t aux;
    uint16_t index;

    aux = ( pHandle->OverVoltageThreshold + pHandle->UnderVoltageThreshold ) / 2u;
    for ( index = 0u; index < pHandle->LowPassFilterBW; index++ )
    {
        pHandle->aBuffer[index] = aux;
    }
    pHandle->_Super.LatestConv = aux;
    pHandle->_Super.AvBusVoltage_d = aux;
    pHandle->index = 0;
}

static uint16_t RVBS_ConvertVbusFiltrered( RDivider_Handle_t * pHandle )
{
    uint16_t hAux;
    uint8_t vindex;
    uint16_t max = 0, min = 0;
    uint32_t tot = 0u;

    for ( vindex = 0; vindex < pHandle->LowPassFilterBW; )
    {
        hAux = RCM_ExecRegularConv(pHandle->convHandle);

        if ( hAux != 0xFFFFu )
        {
            if ( vindex == 0 )
            {
                min = hAux;
                max = hAux;
            }
            else
            {
                if ( hAux < min )
                {
                    min = hAux;
                }
                if ( hAux > max )
                {
                    max = hAux;
                }
            }
            vindex++;

            tot += hAux;
        }
    }

    tot -= max;
    tot -= min;
    return ( uint16_t )( tot / ( pHandle->LowPassFilterBW - 2u ) );
}

/**
  * @brief  It actually performes the Vbus ADC conversion and updates average
  *         value
  * @param  pHandle related RDivider_Handle_t
  * @retval uint16_t Fault code error
  */
__weak uint32_t RVBS_CalcAvVbusFilt( RDivider_Handle_t * pHandle )
{
    uint32_t wtemp;
    uint16_t hAux;
    uint8_t i;

    hAux = RVBS_ConvertVbusFiltrered( pHandle );

    if ( hAux != 0xFFFF )
    {
        pHandle->aBuffer[pHandle->index] = hAux;
        wtemp = 0;
        for ( i = 0; i < pHandle->LowPassFilterBW; i++ )
        {
            wtemp += pHandle->aBuffer[i];
        }
        wtemp /= pHandle->LowPassFilterBW;
        pHandle->_Super.AvBusVoltage_d = ( uint16_t )wtemp;
        pHandle->_Super.LatestConv = hAux;

        if ( pHandle->index < pHandle->LowPassFilterBW - 1 )
        {
            pHandle->index++;
        }
        else
        {
            pHandle->index = 0;
        }
    }

    pHandle->_Super.FaultState = RVBS_CheckFaultState( pHandle );

    return ( pHandle->_Super.FaultState );
}

/**
  * @brief  It actually performes the Vbus ADC conversion and updates average
  *         value
  * @param  pHandle related RDivider_Handle_t
  * @retval uint16_t Fault code error
  */
__weak uint32_t RVBS_CalcAvVbus( RDivider_Handle_t * pHandle )
{
    uint32_t wtemp;
    uint16_t hAux;
    uint8_t i;

    hAux = RCM_ExecRegularConv(pHandle->convHandle);

    if ( hAux != 0xFFFF )
    {
        pHandle->aBuffer[pHandle->index] = hAux;
        wtemp = 0;
    for ( i = 0; i < pHandle->LowPassFilterBW; i++ )
    {
      wtemp += pHandle->aBuffer[i];
    }
    wtemp /= pHandle->LowPassFilterBW;
        pHandle->_Super.AvBusVoltage_d = ( uint16_t )wtemp;
        pHandle->_Super.LatestConv = hAux;

    if ( pHandle->index < pHandle->LowPassFilterBW - 1 )
    {
      pHandle->index++;
    }
    else
    {
      pHandle->index = 0;
    }
    }

    pHandle->_Super.FaultState = RVBS_CheckFaultState( pHandle );

    return ( pHandle->_Super.FaultState );
}

/**
  * @brief  It returns MC_OVER_VOLT, MC_UNDER_VOLT or MC_NO_ERROR depending on
  *         bus voltage and protection threshold values
  * @param  pHandle related RDivider_Handle_t
  * @retval uint16_t Fault code error
  */
/*Under 16V|_____16-20不动作______20V Recover________________24VNormal_____________32 Recover______32-36不动作__________|36V Over*/
uint16_t VBS_Cnt_Compare_High_Threshold = 0;
uint16_t VBS_Cnt_Compare_Low_Threshold = 0;
#define VBS_OVER_TIME   200  /*100MS*/
#define VBS_UNDER_TIME  1500  /*750MS，最大不要超过uint16_t*/
__weak uint32_t RVBS_CheckFaultState( RDivider_Handle_t * pHandle )
{
    static uint32_t fault;

    if ( pHandle->_Super.AvBusVoltage_d > pHandle->OverVoltageThreshold )   /*过压*/
    {
        VBS_Cnt_Compare_High_Threshold ++;
        if(VBS_Cnt_Compare_High_Threshold > VBS_OVER_TIME)  /*连续过压200ms*/
        {
					  VBS_Cnt_Compare_High_Threshold = VBS_OVER_TIME;
            fault = MC_OVER_VOLT;
        }
				else
				{
					  fault = MC_NO_ERROR;
				}
    }
    else if ( pHandle->_Super.AvBusVoltage_d <= pHandle->OverVoltageThreshold *0.89 && \
			        pHandle->_Super.AvBusVoltage_d > pHandle->UnderVoltageThreshold *1.25) //恢复电压 30
    {
        if(VBS_Cnt_Compare_High_Threshold > 0)
        {
            VBS_Cnt_Compare_High_Threshold--;
        }
				if(VBS_Cnt_Compare_Low_Threshold > 0)
				{
					VBS_Cnt_Compare_Low_Threshold--;
				}
				fault = MC_NO_ERROR;  /*因为输入的电流参数已经经历过一次滤波所有为了方便处理这里直接报告无错误*/
    }
    else if ( pHandle->_Super.AvBusVoltage_d < pHandle->UnderVoltageThreshold )
    {
			VBS_Cnt_Compare_Low_Threshold++;
			if(VBS_Cnt_Compare_Low_Threshold > VBS_UNDER_TIME)
			{
				VBS_Cnt_Compare_Low_Threshold = VBS_UNDER_TIME;
        fault = MC_UNDER_VOLT;
			}
			else
			{
				fault = MC_NO_ERROR;
			}
    }

    if ( pHandle->_Super.AvBusVoltage_d > pHandle->OverVoltageThreshold*0.89 )//36V / 32V
    {
//    fault = TURN_ON_R_BRAKE;
    }
    return fault;
}


/**
  * @}
  */

/**
  * @}
  */

/** @} */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/

