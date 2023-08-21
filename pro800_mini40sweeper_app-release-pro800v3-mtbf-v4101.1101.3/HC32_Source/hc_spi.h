#ifndef _HC_SPI_H
#define _HC_SPI_H

#include "hc32_ll.h"

#define BSP_SPI_CS_PORT                 (GPIO_PORT_E)
#define BSP_SPI_CS_PIN                  (GPIO_PIN_03)
#define BSP_SPI_CS_ACTIVE()             GPIO_ResetPins(BSP_SPI_CS_PORT, BSP_SPI_CS_PIN)
#define BSP_SPI_CS_INACTIVE()           GPIO_SetPins(BSP_SPI_CS_PORT, BSP_SPI_CS_PIN)

#define BSP_SPI_SCK_PORT                (GPIO_PORT_E)
#define BSP_SPI_SCK_PIN                 (GPIO_PIN_02)
#define BSP_SPI_SCK_PIN_FUNC            (GPIO_FUNC_40)          /*!< SPI1 SCK */

#define BSP_SPI_MOSI_PORT               (GPIO_PORT_E)           /*!< W25Qxx IO0 */
#define BSP_SPI_MOSI_PIN                (GPIO_PIN_06)
#define BSP_SPI_MOSI_PIN_FUNC           (GPIO_FUNC_41)          /*!< SPI1 MOSI */

#define BSP_SPI_MISO_PORT               (GPIO_PORT_E)           /*!< W25Qxx IO1 */
#define BSP_SPI_MISO_PIN                (GPIO_PIN_05)
#define BSP_SPI_MISO_PIN_FUNC           (GPIO_FUNC_42)          /*!< SPI1 MISO */

#define BSP_SPI_UNIT                    CM_SPI4
#define BSP_SPI_PERIPH_CLK              FCG1_PERIPH_SPI4

#define BSP_SPI_TIMEOUT                 ((HCLK_VALUE / 1000UL))

#define HAL_StatusTypeDef int32_t

void hc_spi_init(void);
int32_t hc_spi_trans(const uint8_t *pu8TxBuf, uint32_t u32Size);
int32_t hc_spi_recv(uint8_t *pu8RxBuf, uint32_t u32Size);
int32_t hc_spi_trans_recv(uint8_t *pu8TxBuf, uint8_t *pu8RxBuf, uint32_t u32Size);
void hc_spi_test(void);

#endif

#if 0
/**
 * @defgroup W25Q64_Size W25Q64 Size
 * @{
 */
#define W25Q64_PAGE_SIZE                (256UL)
#define W25Q64_SECTOR_SIZE              (1024UL * 4UL)
#define W25Q64_BLOCK_SIZE               (1024UL * 64UL)
#define W25Q64_PAGE_PER_SECTOR          (W25Q64_SECTOR_SIZE / W25Q64_PAGE_SIZE)
#define W25Q64_MAX_ADDR                 (0x800000UL)
/**
 * @}
 */

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/

/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/
/**
 * @addtogroup EV_HC32F4A0_LQFP176_W25QXX_Global_Functions
 * @{
 */

void BSP_W25QXX_Init(void);
void BSP_W25QXX_DeInit(void);
int32_t BSP_W25QXX_Write(uint32_t u32Addr, const uint8_t *pu8Data, uint32_t u32NumByteToWrite);
int32_t BSP_W25QXX_Read(uint32_t u32Addr, uint8_t *pu8Data, uint32_t u32NumByteToRead);
int32_t BSP_W25QXX_EraseSector(uint32_t u32Addr);
int32_t BSP_W25QXX_EraseChip(void);

/**
 * @}
 */


/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#endif /* __EV_HC32F4A0_LQFP176_W25QXX_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
