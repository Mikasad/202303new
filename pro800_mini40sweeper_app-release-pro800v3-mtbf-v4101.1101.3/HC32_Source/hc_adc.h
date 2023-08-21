#ifndef _HC_ADC_H
#define _HC_ADC_H

#include "hc32_ll.h"


/* ADC unit instance for this example. */
#define ADC_UNIT                        (CM_ADC3)
#define ADC_PERIPH_CLK                  (FCG3_PERIPH_ADC3)

/* Selects ADC channels that needed. */
#define ADC_IN1                         (ADC_CH10)
#define ADC_IN1_GPIO_Port               (GPIO_PORT_C)
#define ADC_IN1_Pin                     (GPIO_PIN_00)

#define ADC_IN2                         (ADC_CH12)
#define ADC_IN2_GPIO_Port               (GPIO_PORT_C)
#define ADC_IN2_Pin                     (GPIO_PIN_02)

#define ADC_IN3                         (ADC_CH9)
#define ADC_IN3_GPIO_Port               (GPIO_PORT_F)
#define ADC_IN3_Pin                     (GPIO_PIN_03)

#define ADC_IN4                         (ADC_CH14)
#define ADC_IN4_GPIO_Port               (GPIO_PORT_F)
#define ADC_IN4_Pin                     (GPIO_PIN_04)

#define ADC_IN5                         (ADC_CH4)
#define ADC_IN5_GPIO_Port               (GPIO_PORT_A)
#define ADC_IN5_Pin                     (GPIO_PIN_04)

#define ADC_IN6                         (ADC_CH5)
#define ADC_IN6_GPIO_Port               (GPIO_PORT_A)
#define ADC_IN6_Pin                     (GPIO_PIN_05)

#define ADC_SAMPLE_TIME                 30
/* ADC sequence to be used. */
#define ADC_SEQ                         (ADC_SEQ_A)

/* Hard trigger of the specified sequence. */
#define ADC_SEQ_HARDTRIG                (ADC_HARDTRIG_ADTRG_PIN)
#define ADC_SEQ_TRIG_PORT               (GPIO_PORT_E)
#define ADC_SEQ_TRIG_PIN                (GPIO_PIN_07)
#define ADC_SEQ_TRIG_PIN_FUNC           (GPIO_FUNC_1)

/*
 * Definitions of DMA.
 * 'APP_DMA_BLOCK_SIZE': 1~1024, inclusive. 1~16 for ADC1 and ADC2; 1~20 for ADC3.
 * 'APP_DMA_TRANS_COUNT': 0~65535, inclusive. 0: always transmit.
 */
#define DMA_UNIT                        (CM_DMA2)
#define DMA_PERIPH_CLK                  (FCG0_PERIPH_DMA2)
#define ADC1_DMA_CH                     (DMA_CH0)
#define ADC3_DMA_CH                     (DMA_CH2)
#define DMA_AOS_TRIG_SEL                (AOS_DMA1_0)
#define ADC1_DMA_AOS_TRIG_SEL           (AOS_DMA2_0)
#define ADC3_DMA_AOS_TRIG_SEL           (AOS_DMA2_2)

#define DMA_TRANS_CNT                   (0U)
#define ADC1_DMA_BLOCK_SIZE             (ADC_CH5 - ADC_CH4 + 1U)
//#define ADC1_DMA_BLOCK_SIZE             ((ADC_CH5 - ADC_CH4 + 1U)*5)
#define DMA_DATA_WIDTH                  (DMA_DATAWIDTH_16BIT)
#define ADC1_DMA_SRC_ADDR               ((uint32_t)&CM_ADC1->DR4)
#define ADC1_DMA_DEST_ADDR              ((uint32_t)(&g_Adc1DmaData[0U]))

#define ADC3_DMA_BLOCK_SIZE             (ADC_CH14 - ADC_CH9 + 1U)
//#define ADC3_DMA_BLOCK_SIZE             ((ADC_CH14 - ADC_CH9 + 1U)*10)
#define ADC3_DMA_SRC_ADDR               ((uint32_t)&CM_ADC3->DR9)
#define ADC3_DMA_DEST_ADDR              ((uint32_t)(&g_Adc3DmaData[0U]))

#if 1
#define DMA_TRIG_EVT                    (EVT_SRC_ADC1_EOCA)
#else
#define DMA_TRIG_EVT                    (EVT_SRC_ADC3_EOCA)
#endif
#define ADC1_DMA_TRIG_EVT               (EVT_SRC_ADC1_EOCA)
#define ADC3_DMA_TRIG_EVT               (EVT_SRC_ADC3_EOCA)

//#define DMA_INT_TYPE                    (DMA_INT_BTC_CH0)
//#define DMA_INT_SRC                     (INT_SRC_DMA1_BTC0)
//#define DMA_INT_IRQn                    (INT038_IRQn)
//#define DMA_INT_PRIO                    (DDL_IRQ_PRIO_03)
//#define DMA_INT_FLAG                    (DMA_FLAG_BTC_CH0)

#define ADC1_DMA_INT_TYPE               (DMA_INT_BTC_CH0)
#define ADC1_DMA_INT_SRC                (INT_SRC_DMA2_BTC0)
#define ADC1_DMA_INT_IRQn               (INT044_IRQn)
#define ADC1_DMA_INT_PRIO               (DDL_IRQ_PRIO_05)
#define ADC1_DMA_INT_FLAG               (DMA_FLAG_BTC_CH0)

#define ADC3_DMA_INT_TYPE               (DMA_INT_BTC_CH2)
#define ADC3_DMA_INT_SRC                (INT_SRC_DMA2_BTC2)
#define ADC3_DMA_INT_IRQn               (INT045_IRQn)
#define ADC3_DMA_INT_PRIO               (DDL_IRQ_PRIO_05)
#define ADC3_DMA_INT_FLAG               (DMA_FLAG_BTC_CH2)

/* ADC reference voltage. The voltage of pin VREFH. */
#define ADC_VREF                        (3.3F)

/* ADC accuracy(according to the resolution of ADC). */
#define ADC_ACCURACY                    (1UL << 12U)

/* Calculate the voltage(mV). */
#define ADC_CAL_VOL(adcVal)             (uint16_t)((((float32_t)(adcVal) * ADC_VREF) / ((float32_t)ADC_ACCURACY)) * 1000.F)


extern uint16_t g_Adc1DmaData[ADC1_DMA_BLOCK_SIZE];
extern uint16_t g_Adc3DmaData[ADC3_DMA_BLOCK_SIZE];


void hc_adc1_init(void);
void hc_adc3_init(void);
void hc_dma_init(void);
void hc_adc_test(void);

#endif  
