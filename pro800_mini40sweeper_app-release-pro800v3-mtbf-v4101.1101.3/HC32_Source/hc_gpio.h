#ifndef _HC_GPIO_H
#define _HC_GPIO_H

#include "hc32_ll.h"


/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* LED_G Port/Pin definition */
#define BUZZER_GPIO_Port          (GPIO_PORT_G)
#define BUZZER_Pin                (GPIO_PIN_12)
#define ON_RELAY1_GPIO_Port       (GPIO_PORT_E)
#define ON_RELAY1_Pin             (GPIO_PIN_10)
#define ON_RELAY2_GPIO_Port       (GPIO_PORT_E)
#define ON_RELAY2_Pin             (GPIO_PIN_11)
#define ON_RELAY3_GPIO_Port       (GPIO_PORT_E)
#define ON_RELAY3_Pin             (GPIO_PIN_12)
#define ON_RELAY4_GPIO_Port       (GPIO_PORT_E)
#define ON_RELAY4_Pin             (GPIO_PIN_13)
#define CO24V_3_GPIO_Port         (GPIO_PORT_G)
#define CO24V_3_Pin               (GPIO_PIN_07)
#define CO24V_4_GPIO_Port         (GPIO_PORT_G)
#define CO24V_4_Pin               (GPIO_PIN_08)
#define OUTPUT1_GPIO_Port         (GPIO_PORT_D)
#define OUTPUT1_Pin               (GPIO_PIN_03)
#define OUTPUT2_GPIO_Port         (GPIO_PORT_D)
#define OUTPUT2_Pin               (GPIO_PIN_04)
#define CO24V_1_GPIO_Port         (GPIO_PORT_C)
#define CO24V_1_Pin               (GPIO_PIN_09)
#define CO24V_2_GPIO_Port         (GPIO_PORT_G)
#define CO24V_2_Pin               (GPIO_PIN_06)
#define SYS_LED_GPIO_Port         (GPIO_PORT_H)
#define SYS_LED_Pin               (GPIO_PIN_04)
#define DOUT1_GPIO_Port           (GPIO_PORT_H) //实际改为DIN
#define DOUT1_Pin                 (GPIO_PIN_08)
#define DOUT2_GPIO_Port           (GPIO_PORT_H)
#define DOUT2_Pin                 (GPIO_PIN_07)
#define V12_EN_GPIO_Port          (GPIO_PORT_I)
#define V12_EN_Pin                (GPIO_PIN_02)
#define DC5V_EN_1_GPIO_Port       (GPIO_PORT_I)
#define DC5V_EN_1_Pin             (GPIO_PIN_07)
#define V3V3_EN_1_GPIO_Port       (GPIO_PORT_I)
#define V3V3_EN_1_Pin             (GPIO_PIN_08)
#define DC3V3_EN_GPIO_Port        (GPIO_PORT_I)
#define DC3V3_EN_Pin              (GPIO_PIN_10)
#define CO24V_5_GPIO_Port         (GPIO_PORT_E)
#define CO24V_5_Pin               (GPIO_PIN_09)
#define CO24V_6_GPIO_Port         (GPIO_PORT_E)
#define CO24V_6_Pin               (GPIO_PIN_14)
#define CO24V_7_GPIO_Port         (GPIO_PORT_F)
#define CO24V_7_Pin               (GPIO_PIN_08)
#define EN1_GPIO_Port             (GPIO_PORT_E)
#define EN1_Pin                   (GPIO_PIN_07)
#define BRK1_GPIO_Port            (GPIO_PORT_E)
#define BRK1_Pin                  (GPIO_PIN_08)
#define DIR0_GPIO_Port            (GPIO_PORT_F)
#define DIR0_Pin                  (GPIO_PIN_13)
#define EN0_GPIO_Port             (GPIO_PORT_F)
#define EN0_Pin                   (GPIO_PIN_14)
#define BRK0_GPIO_Port            (GPIO_PORT_F)
#define BRK0_Pin                  (GPIO_PIN_15)
#define DIR1_GPIO_Port            (GPIO_PORT_G)
#define DIR1_Pin                  (GPIO_PIN_00)
#define MOTOR_CTR_GPIO_Port       (GPIO_PORT_G)
#define MOTOR_CTR_Pin             (GPIO_PIN_05)
#define CHARGE_SWITCH_GPIO_Port   (GPIO_PORT_G)
#define CHARGE_SWITCH_Pin         (GPIO_PIN_10)
#define DC5V_EN1_GPIO_Port        (GPIO_PORT_G)
#define DC5V_EN1_Pin              (GPIO_PIN_11)
#define DC5V_EN_GPIO_Port         (GPIO_PORT_B)
#define DC5V_EN_Pin               (GPIO_PIN_07)
#define HARD_VER1_GPIO_Port       (GPIO_PORT_H)
#define HARD_VER1_Pin             (GPIO_PIN_03)
#define ALM_1_GPIO_Port           (GPIO_PORT_A)
#define ALM_1_Pin                 (GPIO_PIN_06)
#define ALM_0_GPIO_Port           (GPIO_PORT_A)
#define ALM_0_Pin                 (GPIO_PIN_08)
#define HARD_VER2_GPIO_Port       (GPIO_PORT_I)
#define HARD_VER2_Pin             (GPIO_PIN_01)
#define TLY_RST_GPIO_Port         (GPIO_PORT_H)
#define TLY_RST_Pin               (GPIO_PIN_02)

#define RS485_CONTROL1_GPIO_Port  (GPIO_PORT_H)
#define RS485_CONTROL1_Pin        (GPIO_PIN_09)

#define RS485_CONTROL2_GPIO_Port  (GPIO_PORT_I)
#define RS485_CONTROL2_Pin        (GPIO_PIN_03)

#define RS485_CONTROL3_GPIO_Port  (GPIO_PORT_I)
#define RS485_CONTROL3_Pin        (GPIO_PIN_05)

#define CONTROL_485_1_GPIO_Port   RS485_CONTROL1_GPIO_Port
#define CONTROL_485_1_Pin         RS485_CONTROL1_Pin
#define CONTROL_485_2_GPIO_Port   RS485_CONTROL2_GPIO_Port
#define CONTROL_485_2_Pin         RS485_CONTROL2_Pin
#define CONTROL_485_3_GPIO_Port   RS485_CONTROL3_GPIO_Port
#define CONTROL_485_3_Pin         RS485_CONTROL3_Pin

#define DIN1_GPIO_Port            (GPIO_PORT_B)
#define DIN1_Pin                  (GPIO_PIN_01)
#define DIN2_GPIO_Port            (GPIO_PORT_E)
#define DIN2_Pin                  (GPIO_PIN_15)
#define DIN3_GPIO_Port            (GPIO_PORT_H)
#define DIN3_Pin                  (GPIO_PIN_08)
#define DIN4_GPIO_Port            (GPIO_PORT_H)
#define DIN4_Pin                  (GPIO_PIN_07)

#define DAC_OUT1_GPIO_Port        (GPIO_PORT_A)
#define DAC_OUT1_Pin              (GPIO_PIN_04)
#define DAC_OUT2_GPIO_Port        (GPIO_PORT_A)
#define DAC_OUT2_Pin              (GPIO_PIN_05)
#define ADC_IN1_GPIO_Port         (GPIO_PORT_C)
#define ADC_IN1_Pin               (GPIO_PIN_00)
#define ADC_IN2_GPIO_Port         (GPIO_PORT_C)
#define ADC_IN2_Pin               (GPIO_PIN_02)
#define ADC_IN3_GPIO_Port         (GPIO_PORT_F)
#define ADC_IN3_Pin               (GPIO_PIN_03)
#define ADC_IN4_GPIO_Port         (GPIO_PORT_F)
#define ADC_IN4_Pin               (GPIO_PIN_04)

#define IN0_GPIO_Port             (GPIO_PORT_H)
#define IN0_Pin                   (GPIO_PIN_15)
#define IN1_GPIO_Port             (GPIO_PORT_G)
#define IN1_Pin                   (GPIO_PIN_03)
#define IN2_GPIO_Port             (GPIO_PORT_G)
#define IN2_Pin                   (GPIO_PIN_02)
#define IN3_GPIO_Port             (GPIO_PORT_D)
#define IN3_Pin                   (GPIO_PIN_10)
#define IN4_GPIO_Port             (GPIO_PORT_D)
#define IN4_Pin                   (GPIO_PIN_11)
#define IN5_GPIO_Port             (GPIO_PORT_C)
#define IN5_Pin                   (GPIO_PIN_08)
#define IN6_GPIO_Port             (GPIO_PORT_H)
#define IN6_Pin                   (GPIO_PIN_14)
#define IN7_GPIO_Port             (GPIO_PORT_H)
#define IN7_Pin                   (GPIO_PIN_13)
#define IN8_GPIO_Port             (GPIO_PORT_D)
#define IN8_Pin                   (GPIO_PIN_12)
#define IN9_GPIO_Port             (GPIO_PORT_D)
#define IN9_Pin                   (GPIO_PIN_13)

#define IN11_GPIO_Port            (GPIO_PORT_D)
#define IN11_Pin                  (GPIO_PIN_15)
#define IN10_GPIO_Port            (GPIO_PORT_D)
#define IN10_Pin                  (GPIO_PIN_14)
#define IN12_GPIO_Port            (GPIO_PORT_F)
#define IN12_Pin                  (GPIO_PIN_09)
#define IN13_GPIO_Port            (GPIO_PORT_F)
#define IN13_Pin                  (GPIO_PIN_10)
#define IN14_GPIO_Port            (GPIO_PORT_F)
#define IN14_Pin                  (GPIO_PIN_11)
#define IN15_GPIO_Port            (GPIO_PORT_F)
#define IN15_Pin                  (GPIO_PIN_12)
#define STOP_GPIO_Port            (GPIO_PORT_B)
#define STOP_Pin                  (GPIO_PIN_15)
#define POWER_UP_GPIO_Port        (GPIO_PORT_G)
#define POWER_UP_Pin              (GPIO_PIN_04)
#define MCU_KEY_GPIO_Port         (GPIO_PORT_G)
#define MCU_KEY_Pin               (GPIO_PIN_01)
/* LED toggle definition */
#define BUZZER_TOGGLE()           (GPIO_TogglePins(BUZZER_PORT, BUZZER_PIN))
#define BUZZER_SET()              (GPIO_SetPins(BUZZER_GPIO_Port, BUZZER_Pin))
#define BUZZER_RESET()            (GPIO_ResetPins(BUZZER_GPIO_Port, BUZZER_Pin))
#define RELAY1_SET()              (GPIO_SetPins(ON_RELAY1_GPIO_Port, ON_RELAY1_Pin))
#define RELAY1_RESET()            (GPIO_ResetPins(ON_RELAY1_GPIO_Port, ON_RELAY1_Pin))
#define RELAY2_SET()              (GPIO_SetPins(ON_RELAY2_GPIO_Port, ON_RELAY2_Pin))
#define RELAY2_RESET()            (GPIO_ResetPins(ON_RELAY2_GPIO_Port, ON_RELAY2_Pin))
#define RELAY3_SET()              (GPIO_SetPins(ON_RELAY3_GPIO_Port, ON_RELAY3_Pin))
#define RELAY3_RESET()            (GPIO_ResetPins(ON_RELAY3_GPIO_Port, ON_RELAY3_Pin))
#define RELAY4_SET()              (GPIO_SetPins(ON_RELAY4_GPIO_Port, ON_RELAY4_Pin))
#define RELAY4_RESET()            (GPIO_ResetPins(ON_RELAY4_GPIO_Port, ON_RELAY4_Pin))
#define DC24V_OUT3_GND_SET()      (GPIO_SetPins(CO24V_3_GPIO_Port, CO24V_3_Pin))
#define DC24V_OUT3_GND_RESET()    (GPIO_ResetPins(CO24V_3_GPIO_Port, CO24V_3_Pin))
#define DC24V_OUT4_GND_SET()      (GPIO_SetPins(CO24V_4_GPIO_Port, CO24V_4_Pin))
#define DC24V_OUT4_GND_RESET()    (GPIO_ResetPins(CO24V_4_GPIO_Port, CO24V_4_Pin))
#define DC24V_OUT1_SET()          (GPIO_SetPins(OUTPUT1_GPIO_Port, OUTPUT1_Pin))
#define DC24V_OUT1_RESET()        (GPIO_ResetPins(OUTPUT1_GPIO_Port, OUTPUT1_Pin))
#define DC24V_OUT2_SET()          (GPIO_SetPins(OUTPUT2_GPIO_Port, OUTPUT2_Pin))
#define DC24V_OUT2_RESET()        (GPIO_ResetPins(OUTPUT2_GPIO_Port, OUTPUT2_Pin))
#define DC24V_OUT3_SET()          (GPIO_SetPins(CO24V_1_GPIO_Port, CO24V_1_Pin))
#define DC24V_OUT3_RESET()        (GPIO_ResetPins(CO24V_1_GPIO_Port, CO24V_1_Pin))
#define DC24V_OUT4_SET()          (GPIO_SetPins(CO24V_2_GPIO_Port, CO24V_2_Pin))
#define DC24V_OUT4_RESET()        (GPIO_ResetPins(CO24V_2_GPIO_Port, CO24V_2_Pin))
#define LED_SET()                 (GPIO_SetPins(SYS_LED_GPIO_Port, SYS_LED_Pin))
#define LED_RESET()               (GPIO_ResetPins(SYS_LED_GPIO_Port, SYS_LED_Pin))
#define OUT1_IO_SET()             (GPIO_SetPins(DOUT1_GPIO_Port, DOUT1_Pin))
#define OUT1_IO_RESET()           (GPIO_ResetPins(DOUT1_GPIO_Port, DOUT1_Pin))
#define OUT2_IO_SET()             (GPIO_SetPins(DOUT2_GPIO_Port, DOUT2_Pin))
#define OUT2_IO_RESET()           (GPIO_ResetPins(DOUT2_GPIO_Port, DOUT2_Pin))
#define V12_EN_SET()              (GPIO_SetPins(V12_EN_GPIO_Port, V12_EN_Pin))
#define V12_EN_RESET()            (GPIO_ResetPins(V12_EN_GPIO_Port, V12_EN_Pin))
#define DC5V_EN_1_SET()           (GPIO_SetPins(DC5V_EN_1_GPIO_Port, DC5V_EN_1_Pin))
#define DC5V_EN_1_RESET()         (GPIO_ResetPins(DC5V_EN_1_GPIO_Port, DC5V_EN_1_Pin))
#define DC3V3_EN_1_SET()          (GPIO_SetPins(V3V3_EN_1_GPIO_Port, V3V3_EN_1_Pin))
#define DC3V3_EN_1_RESET()        (GPIO_ResetPins(V3V3_EN_1_GPIO_Port, V3V3_EN_1_Pin))
#define DC3V3_EN_SET()            (GPIO_SetPins(DC3V3_EN_GPIO_Port, DC3V3_EN_Pin))
#define DC3V3_EN_RESET()          (GPIO_ResetPins(DC3V3_EN_GPIO_Port, DC3V3_EN_Pin))
#define DIN1_IO_READ()            (GPIO_ReadInputPins(DIN1_GPIO_Port, DIN1_Pin))
#define DIN2_IO_READ()            (GPIO_ReadInputPins(DIN2_GPIO_Port, DIN2_Pin))
#define IN5_IO_READ()             (GPIO_ReadInputPins(DAC_OUT1_GPIO_Port, DAC_OUT1_Pin))
#define IN6_IO_READ()             (GPIO_ReadInputPins(DAC_OUT2_GPIO_Port, DAC_OUT2_Pin))

#define IN10_READ()               (GPIO_ReadInputPins(IN10_GPIO_Port, IN10_Pin))
#define IN11_READ()               (GPIO_ReadInputPins(IN11_GPIO_Port, IN11_Pin))
#define IN12_READ()               (GPIO_ReadInputPins(IN12_GPIO_Port, IN12_Pin))
#define IN13_READ()               (GPIO_ReadInputPins(IN13_GPIO_Port, IN13_Pin))
#define IN14_READ()               (GPIO_ReadInputPins(IN14_GPIO_Port, IN14_Pin))
#define IN15_READ()               (GPIO_ReadInputPins(IN15_GPIO_Port, IN15_Pin))
#define STOP_READ()               (GPIO_ReadInputPins(STOP_GPIO_Port, STOP_Pin))
#define POWER_UP_READ()           (GPIO_ReadInputPins(POWER_UP_GPIO_Port, POWER_UP_Pin))
#define MCU_KEY_READ()            (GPIO_ReadInputPins(MCU_KEY_GPIO_Port, MCU_KEY_Pin))

#define DLY_MS                    (2000UL)

#define HAL_Delay DDL_DelayMS
#define HAL_GPIO_ReadPin GPIO_ReadInputPins
#define HAL_GPIO_TogglePin GPIO_TogglePins

#define GPIO_PIN_RESET  PIN_RESET 
#define GPIO_PIN_SET    PIN_SET

void HAL_GPIO_WritePin( uint32_t port ,uint32_t pin  ,uint32_t PinState  );
void hc_gpio_init(void);
void hc_gpio_test(void);

#endif  
