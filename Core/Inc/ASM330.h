/*
 * ASM330.h
 *
 *  Created on: Nov 25, 2023
 *      Author: thean
 */

#ifndef INC_ASM330_H_
#define INC_ASM330_H_

#include "asm330lhhx_reg.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"

typedef struct {
	SPI_HandleTypeDef *hspi;
	stmdev_ctx_t dev_ctx;
	asm330lhhx_odr_xl_t accel_odr;
	asm330lhhx_fs_xl_t accel_scale;
	asm330lhhx_odr_g_t gyro_odr;
	asm330lhhx_fs_g_t gyro_scale;
	GPIO_TypeDef *CS_GPIO_Port;
	uint16_t CS_Pin;
} ASM330_handle;

#define BOOT_TIME 10		// ms

uint8_t ASM330_Init(ASM330_handle *asm330);
uint8_t ASM330_readAccel(ASM330_handle *asm330, float* accel);
uint8_t ASM330_readGyro(ASM330_handle *asm330, float* gyro);

#endif /* INC_ASM330_H_ */
