/*
 * SD.h
 *
 *  Created on: 21 Jul. 2023
 *      Author: Angus McLennan
 */

#ifndef INC_SD_H_
#define INC_SD_H_

#include "fatfs.h"
#include "stdio.h"
#include "string.h"
#include <stdbool.h>

/* DEFINITIONS */
typedef struct {
	float log_frequency;
	bool flash_good;
	bool flash_logging_enabled;
} SD_Handle_t;

extern SD_Handle_t SD_card;

FRESULT SD_init();
FRESULT SD_mk_root_dir();
void SD_Error_Handler();
FRESULT SD_write_accelerometer_data(uint32_t time_uS, float acc1X, float acc1Y, float acc1Z, float acc2X, float acc2Y, float acc2Z);
FRESULT SD_write_gyroscope_data(uint32_t time_uS, float gyro1X, float gyro1Y, float gyro1Z, float gyro2X, float gyro2Y, float gyro2Z);
FRESULT SD_write_magnetometer_data(uint32_t time_uS, float magX, float magY, float magZ);
FRESULT SD_write_barometer_data(uint32_t time_uS, float altitude, float pressure, float temperature);
FRESULT SD_write_GPS_data(uint32_t time_uS, int time_hours, int time_minutes, int time_seconds, int date_day, int date_month, int date_year, int hour_offset, int minute_offset, float latitude, float longitude, float altitude, int fix_quality, int satelites_tracked);
FRESULT SD_write_system_state_data(uint32_t time_uS, uint8_t flight_state, uint8_t drogue_ematch_status, uint8_t main_ematch_status, uint32_t launch_time, uint32_t drogue_deploy_time, float drogue_deploy_altitude, uint32_t main_deploy_time, float main_deploy_altitude, uint32_t landing_time, float landing_altitude, float battery_voltage);
FRESULT SD_write_ekf_data(uint32_t time_uS, float qu1, float qu2, float qu3, float qu4);
FRESULT SD_write_headers();
FRESULT SD_write_binary_stream(uint8_t *buffer, size_t len);
FRESULT SD_get_free_space_kB(float* kBytes_free);;
FRESULT SD_erase_disk();
#endif /* INC_SD_H_ */
