
//stm32头文件
//APP
#include "flash_op.h"
#include "drv_ota.h"
//#include "drv_imu_ota.h"



static OTA_INFO s_ota_info = {0};

/************************************************************************************************************************************
																变量定义
*************************************************************************************************************************************/
extern PhantasMcuUpgradeCommand PhantasMcuUpgrade_CMD;
extern PhantasMcuUpgradePackage PhantasMcuUpgradePackage_CMD;


static OTA_STATUS get_ota_status(void)
{
	return s_ota_info.status;
}

static void set_ota_status(OTA_STATUS status)
{
	s_ota_info.status = status;
}





/************************************************************************************************************************************
											对外提供的数据、状态、配置、控制接口定义
*************************************************************************************************************************************/
uint32_t get_phantas_mcu_upgrade_command_status(OTA_TARGET target)
{
//	if (MAIN_MCU == target)
		return get_ota_status();
//	else if (IMU == target)
//		return get_imu_ota_response_state();
}

void set_phantas_mcu_upgrade_command_status(OTA_TARGET target, OTA_STATUS status)
{
//	if (MAIN_MCU == target)
		set_ota_status(status);
//	else if (IMU == target)
//		set_imu_ota_response_state(status);
}

uint32_t get_phantas_mcu_upgrade_package_status(OTA_TARGET target)
{
//	if (MAIN_MCU == target)
		return get_ota_status();
//	else if (IMU == target)
//		return get_imu_ota_response_state();
}

void set_phantas_mcu_upgrade_package_status(OTA_TARGET target, OTA_STATUS status)
{
//	if (MAIN_MCU == target)
		set_ota_status(status);
//	else if (IMU == target)
//		set_imu_ota_response_state(status);
}





bool mcu_types_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg)
{
	UInt32Pair uint32Pair = {0,0};

	uint32Pair.key = MAIN_MCU;
	uint32Pair.value = MCU_S_V4;
 
	if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
	return false;

	uint32Pair.key = SUB_MCU;
	uint32Pair.value = MCU_GD;
	if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
	return false;	

	uint32Pair.key = IMU;
	uint32Pair.value = JC310;
	if((!pb_encode_tag_for_field(stream, field)) || (!pb_encode_submessage(stream, UInt32Pair_fields, &uint32Pair)))
	return false;	
	
	return true;
}



