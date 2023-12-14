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

/* DEFINITIONS */
#define LOG_FREQUENCY 100 // Hz

bool flash_good;

FRESULT SD_init();
FRESULT SD_mk_root_dir();
void SD_Error_Handler();
FRESULT SD_write_accelerometer_data(uint32_t time_uS, float accX, float accY, float accZ);
FRESULT SD_write_gyroscope_data(uint32_t time_uS, float gyroX, float gyroY, float gyroZ);
FRESULT SD_write_magnetometer_data(uint32_t time_uS, float magX, float magY, float magZ);
FRESULT SD_write_barometer_data(uint32_t time_uS, float altitude, float pressure, float temperature);
FRESULT SD_write_GPS_data(uint32_t time_uS, int time_hours, int time_minutes, int time_seconds, int date_day, int date_month, int date_year, int hour_offset, int minute_offset, float latitude, float longitude, float altitude, int fix_quality, int satelites_tracked);
FRESULT SD_write_system_state_data(uint32_t time_uS, uint8_t tranmsitting_gps, int flight_state, float starting_altitude, uint32_t launch_time, uint8_t controller_saturated);
FRESULT SD_write_ekf_data(uint32_t time_uS, float qu1, float qu2, float qu3, float qu4);
FRESULT SD_write_headers();
FRESULT SD_write_binary_stream(uint8_t *buffer, size_t len);
#endif /* INC_SD_H_ */
