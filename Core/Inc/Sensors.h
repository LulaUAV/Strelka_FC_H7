/*
 * Sensors.h
 *
 *  Created on: Nov 26, 2022
 *      Author: Angus McLennan
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_
#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"

#define ADC_VREF 3.3f
#define ADC_RESOLUTION_BITS 12

/*
 * Data structure to hold BMX055 data
 * Data elements array for each sensor:
 * Data X, Data Y, Data Z
 */
typedef struct {
	float accel[3];
	float gyro[3];
	float mag[3];
	bool accel_updated;
	bool gyro_updated;
	bool mag_updated;
}BMX055_Data_Handle;

typedef struct {
	float pressure;
	float temperature;
	float altitude;
}MS5611_Data_Handle;

typedef struct {
	float accel[3];
	float gyro[3];
	bool accel_updated;
	bool gyro_updated;
}ASM330_Data_Handle;

#define GPS_Buff_Size 1024
typedef struct {
	uint8_t gps_buffer[GPS_Buff_Size];
}GPS_Data_Handle;

void init_adc(ADC_HandleTypeDef* hadc);
float get_battery_voltage(ADC_HandleTypeDef* hadc);

#endif /* INC_SENSORS_H_ */
