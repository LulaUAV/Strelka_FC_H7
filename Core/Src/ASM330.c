/*
 * ASM330.c
 *
 *  Created on: Nov 25, 2023
 *      Author: thean
 */

#include "ASM330.h"

GPIO_TypeDef *CS_up_GPIO_Port;
uint16_t CS_up_Pin;

uint8_t ASM330_Init(ASM330_handle *asm330) {
	static uint8_t whoamI, rst;

	CS_up_GPIO_Port = asm330->CS_GPIO_Port;
	CS_up_Pin = asm330->CS_Pin;

	asm330->dev_ctx.write_reg = platform_write;
	asm330->dev_ctx.read_reg = platform_read;
	asm330->dev_ctx.handle = asm330->hspi;

	osDelay(pdMS_TO_TICKS(BOOT_TIME));

	/* Check device ID. */
	asm330lhhx_device_id_get(&asm330->dev_ctx, &whoamI);
	if (whoamI != ASM330LHHX_ID)
		return 1;

	/* Restore default configuration. */
	asm330lhhx_reset_set(&asm330->dev_ctx, PROPERTY_ENABLE);
	do {
		asm330lhhx_reset_get(&asm330->dev_ctx, &rst);
	} while (rst);

	/* Disable I3C interface. */
	asm330lhhx_i3c_disable_set(&asm330->dev_ctx, ASM330LHHX_I3C_DISABLE);

	/* Enable Block Data Update. */
	asm330lhhx_block_data_update_set(&asm330->dev_ctx, PROPERTY_ENABLE);

	/* Set Output Data Rate. */
	asm330lhhx_xl_data_rate_set(&asm330->dev_ctx, asm330->accel_odr);
	asm330lhhx_gy_data_rate_set(&asm330->dev_ctx, asm330->gyro_odr);

	/* Set full scale. */
	asm330lhhx_xl_full_scale_set(&asm330->dev_ctx, asm330->accel_scale);
	asm330lhhx_gy_full_scale_set(&asm330->dev_ctx, asm330->gyro_scale);

	/* Set INT1 to accelerometer data ready */
	asm330lhhx_pin_int1_route_get(&asm330->dev_ctx, &int1_route);
	int1_route.int1_ctrl.int1_drdy_xl = PROPERTY_ENABLE;
	asm330lhhx_pin_int1_route_set(&asm330->dev_ctx, &int1_route);

	/* Set INT2 to gyroscope data ready */
	asm330lhhx_pin_int2_route_get(&asm330->dev_ctx, &int2_route);
	int2_route.int2_ctrl.int2_drdy_g = PROPERTY_ENABLE;
	asm330lhhx_pin_int2_route_set(&asm330->dev_ctx, &int2_route);

	return 0;
}

uint8_t ASM330_readAccel(ASM330_handle *asm330, float *accel) {
	static int16_t data_raw_acceleration[3];
	asm330lhhx_acceleration_raw_get(&asm330->dev_ctx, data_raw_acceleration);

	switch (asm330->accel_scale) {
	case ASM330LHHX_2g:
		accel[0] = asm330lhhx_from_fs2g_to_mg(data_raw_acceleration[0]) / 1000.0;
		accel[1] = asm330lhhx_from_fs2g_to_mg(data_raw_acceleration[1]) / 1000.0;
		accel[2] = asm330lhhx_from_fs2g_to_mg(data_raw_acceleration[2]) / 1000.0;
		return 1;
	case ASM330LHHX_4g:
		accel[0] = asm330lhhx_from_fs4g_to_mg(data_raw_acceleration[0]) / 1000.0;
		accel[1] = asm330lhhx_from_fs4g_to_mg(data_raw_acceleration[1]) / 1000.0;
		accel[2] = asm330lhhx_from_fs4g_to_mg(data_raw_acceleration[2]) / 1000.0;
		return 1;
	case ASM330LHHX_8g:
		accel[0] = asm330lhhx_from_fs8g_to_mg(data_raw_acceleration[0]) / 1000.0;
		accel[1] = asm330lhhx_from_fs8g_to_mg(data_raw_acceleration[1]) / 1000.0;
		accel[2] = asm330lhhx_from_fs8g_to_mg(data_raw_acceleration[2]) / 1000.0;
		return 1;
	case ASM330LHHX_16g:
		accel[0] = asm330lhhx_from_fs16g_to_mg(data_raw_acceleration[0]) / 1000.0;
		accel[1] = asm330lhhx_from_fs16g_to_mg(data_raw_acceleration[1]) / 1000.0;
		accel[2] = asm330lhhx_from_fs16g_to_mg(data_raw_acceleration[2]) / 1000.0;
		return 1;
	}
}

uint8_t ASM330_readGyro(ASM330_handle *asm330, float *gyro) {
	static int16_t data_raw_angular_rate[3];
	asm330lhhx_angular_rate_raw_get(&asm330->dev_ctx, data_raw_angular_rate);

	switch (asm330->gyro_scale) {
	case ASM330LHHX_125dps:
		gyro[0] = asm330lhhx_from_fs125dps_to_mdps(data_raw_angular_rate[0]) / 1000.0;
		gyro[1] = asm330lhhx_from_fs125dps_to_mdps(data_raw_angular_rate[1]) / 1000.0;
		gyro[2] = asm330lhhx_from_fs125dps_to_mdps(data_raw_angular_rate[2]) / 1000.0;
		return 1;
	case ASM330LHHX_250dps:
		gyro[0] = asm330lhhx_from_fs250dps_to_mdps(data_raw_angular_rate[0]) / 1000.0;
		gyro[1] = asm330lhhx_from_fs250dps_to_mdps(data_raw_angular_rate[1]) / 1000.0;
		gyro[2] = asm330lhhx_from_fs250dps_to_mdps(data_raw_angular_rate[2]) / 1000.0;
		return 1;
	case ASM330LHHX_500dps:
		gyro[0] = asm330lhhx_from_fs500dps_to_mdps(data_raw_angular_rate[0]) / 1000.0;
		gyro[1] = asm330lhhx_from_fs500dps_to_mdps(data_raw_angular_rate[1]) / 1000.0;
		gyro[2] = asm330lhhx_from_fs500dps_to_mdps(data_raw_angular_rate[2]) / 1000.0;
		return 1;
	case ASM330LHHX_1000dps:
		gyro[0] = asm330lhhx_from_fs1000dps_to_mdps(data_raw_angular_rate[0]) / 1000.0;
		gyro[1] = asm330lhhx_from_fs1000dps_to_mdps(data_raw_angular_rate[1]) / 1000.0;
		gyro[2] = asm330lhhx_from_fs1000dps_to_mdps(data_raw_angular_rate[2]) / 1000.0;
		return 1;
	case ASM330LHHX_2000dps:
		gyro[0] = asm330lhhx_from_fs2000dps_to_mdps(data_raw_angular_rate[0]) / 1000.0;
		gyro[1] = asm330lhhx_from_fs2000dps_to_mdps(data_raw_angular_rate[1]) / 1000.0;
		gyro[2] = asm330lhhx_from_fs2000dps_to_mdps(data_raw_angular_rate[2]) / 1000.0;
		return 1;
	case ASM330LHHX_4000dps:
		gyro[0] = asm330lhhx_from_fs4000dps_to_mdps(data_raw_angular_rate[0]) / 1000.0;
		gyro[1] = asm330lhhx_from_fs4000dps_to_mdps(data_raw_angular_rate[1]) / 1000.0;
		gyro[2] = asm330lhhx_from_fs4000dps_to_mdps(data_raw_angular_rate[2]) / 1000.0;
		return 1;
	}
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(handle, &reg, 1, 1000);
	HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
	return 0;
}
