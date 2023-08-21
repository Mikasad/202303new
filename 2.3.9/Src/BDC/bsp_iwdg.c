#include "bsp_iwdg.h"
#include "function.h"
IWDG_HandleTypeDef hiwdg;
/*------------------------------------------------
Function:看门狗初始化
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void MX_IWDG_Init(uint8_t prv,uint16_t rlv)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = prv;
    hiwdg.Init.Reload = rlv;
    HAL_IWDG_Init(&hiwdg);
}
/*------------------------------------------------
Function:ADC模拟看门狗回调函数
Input   :No
Output  :No
Explain :No
------------------------------------------------*/
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
    if(__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_AWD))
    {
        __HAL_ADC_DISABLE_IT(&hadc1,ADC_IT_AWD);
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_AWD);
//////			//需要停止电机
    }
    __HAL_ADC_ENABLE_IT(&hadc1,ADC_IT_AWD);
}
