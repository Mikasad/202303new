
#include "hc_can2.h"

/**
 * @brief can2口 初始化
 */
int16_t bsp_can2_init(void)
{
    bms_can2_rx_quene_handle = xQueueCreate(8, sizeof(stc_can_rx_frame_t));
    if (bms_can2_rx_quene_handle == NULL) 
    {
        printf("can2 Create queue error.\r\n");
        return -1;
    }

    can2_config_init();
		
    return 0;
}
