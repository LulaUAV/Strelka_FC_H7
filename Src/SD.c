/*
 * SD.c
 *
 *  Created on: 21 Jul. 2023
 *      Author: Angus McLennan
 */

#include "SD.h"

char directory_name[] = "/DATA000";
char GPSDir[] = "gps.csv";
char baroDir[] = "baro.csv";
char accelDir[] = "accel.csv";
char gyroDir[] = "gyro.csv";
char magDir[] = "mag.csv";
char systemStateDir[] = "sys.csv";
char ekfDir[] = "ekf.csv";
char streamDir[] = "stream.dat";
const TickType_t xFrequency;

FRESULT SD_init() {
	/* Flash memory file system config */
	uint8_t rtext[_MAX_SS];/* File read buffer */
	FRESULT res = f_mount(&SDFatFS, (TCHAR const*) SDPath, 0);
	if (res != FR_OK) {
		if (res == FR_NO_FILESYSTEM) {
			res = f_mkfs((TCHAR const*) SDPath, FM_ANY, 0, rtext, sizeof(rtext));
			if (res != FR_OK) {
				SD_Error_Handler();
			}
		} else
			SD_Error_Handler();
	}

	res = SD_mk_root_dir();
	if (res != FR_OK) {
		SD_Error_Handler();
	}

	return res;
}

FRESULT SD_mk_root_dir() {
	// Make data directory
	for (int i = 0; i < 999; i++) {
		FILINFO dir_info;
		FRESULT fr;
		fr = f_stat(directory_name, &dir_info);
		if (fr == FR_OK) {
			directory_name[strlen(directory_name) - 3] = '\0';
			sprintf(directory_name, "%s%03d", directory_name, i);
		} else {
//			sprintf(directory_name, "%s%03d", directory_name, i);
//			directory_name[strlen(directory_name) - 3] = '\0';
			break;
		}
	}
	FRESULT res = f_mkdir(directory_name);
	return res;
}

void SD_Error_Handler() {
	while (1) {
		osDelay(100000);
	}
}

FRESULT SD_write_headers() {
	char fname[32];
	uint32_t byteswritten;
	sprintf(fname, "%s/%s", directory_name, accelDir);
	FRESULT res = f_open(&SDFile, fname, FA_OPEN_APPEND | FA_WRITE);
	osDelay(10);
	if (res != FR_OK) {
		Error_Handler();
	}
	uint8_t accel_header[] = "timestamp,accX,accY,accZ\n";
	res = f_write(&SDFile, accel_header, sizeof(accel_header), (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		Error_Handler();
	} else {

		f_close(&SDFile);
	}
	osDelay(10);

	sprintf(fname, "%s/%s", directory_name, gyroDir);
	res = f_open(&SDFile, fname, FA_OPEN_APPEND | FA_WRITE);
	osDelay(10);
	if (res != FR_OK) {
		Error_Handler();
	}
	char gyro_header[] = "timestamp,gyroX,gyroY,gyroZ\n";
	res = f_write(&SDFile, gyro_header, sizeof(gyro_header), (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		Error_Handler();
	} else {

		f_close(&SDFile);
	}
	osDelay(10);

	sprintf(fname, "%s/%s", directory_name, magDir);
	res = f_open(&SDFile, fname, FA_OPEN_APPEND | FA_WRITE);
	osDelay(10);
	if (res != FR_OK) {
		Error_Handler();
	}
	char mag_header[] = "timestamp,magX,magY,magZ\n";
	res = f_write(&SDFile, mag_header, sizeof(mag_header), (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		Error_Handler();
	} else {

		f_close(&SDFile);
	}
	osDelay(10);

	sprintf(fname, "%s/%s", directory_name, baroDir);
	res = f_open(&SDFile, fname, FA_OPEN_APPEND | FA_WRITE);
	osDelay(10);
	if (res != FR_OK) {
		Error_Handler();
	}
	char baro_header[] = "timestamp,altitude,pressure,temperature\n";
	res = f_write(&SDFile, baro_header, sizeof(baro_header), (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		Error_Handler();
	} else {

		f_close(&SDFile);
	}
	osDelay(10);

	return res;
}

FRESULT SD_write_accelerometer_data(uint32_t time_uS, float accX, float accY, float accZ) {
	char accel_fname[32];
	uint8_t write_data[_MAX_SS];
	uint32_t byteswritten;
	sprintf(accel_fname, "%s/%s", directory_name, accelDir);
	FRESULT res = f_open(&SDFile, accel_fname, FA_OPEN_APPEND | FA_WRITE);

	if (res != FR_OK) {
		Error_Handler();
	}

	//Write to the text file
	size_t sz = snprintf((char*) write_data, sizeof(write_data), "%.0lu,%.3f,%.3f,%.3f\n", time_uS, accX, accY, accZ);

	res = f_write(&SDFile, write_data, sz, (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		Error_Handler();
	} else {

		f_close(&SDFile);
	}
	return res;
}

FRESULT SD_write_gyroscope_data(uint32_t time_uS, float gyroX, float gyroY, float gyroZ) {
	uint8_t write_data[_MAX_SS];
	uint32_t byteswritten;
	char gyro_fname[32];
	sprintf(gyro_fname, "%s/%s", directory_name, gyroDir);
	FRESULT res = f_open(&SDFile, gyro_fname,
	FA_OPEN_APPEND | FA_WRITE);

	if (res != FR_OK) {
		Error_Handler();
	}

	//Write to the text file
	size_t sz = snprintf((char*) write_data, sizeof(write_data), "%.0lu,%.3f,%.3f,%.3f\n", time_uS, gyroX, gyroY, gyroZ);

	res = f_write(&SDFile, write_data, sz, (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		Error_Handler();
	} else {

		f_close(&SDFile);
	}
	return res;
}

FRESULT SD_write_magnetometer_data(uint32_t time_uS, float magX, float magY, float magZ) {
	uint8_t write_data[_MAX_SS];
	uint32_t byteswritten;
	char mag_fname[32];
	sprintf(mag_fname, "%s/%s", directory_name, magDir);
	FRESULT res = f_open(&SDFile, mag_fname, FA_OPEN_APPEND | FA_WRITE);

	if (res != FR_OK) {
		Error_Handler();
	}

	//Write to the text file
	size_t sz = snprintf((char*) write_data, sizeof(write_data), "%.0lu,%.3f,%.3f,%.3f\n", time_uS, magX, magY, magZ);

	res = f_write(&SDFile, write_data, sz, (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		Error_Handler();
	} else {

		f_close(&SDFile);
	}
	return res;
}

FRESULT SD_write_barometer_data(uint32_t time_uS, float altitude, float pressure, float temperature) {
	uint8_t write_data[_MAX_SS];
	uint32_t byteswritten;
	char baro_fname[32];
	sprintf(baro_fname, "%s/%s", directory_name, baroDir);
	FRESULT res = f_open(&SDFile, baro_fname, FA_OPEN_APPEND | FA_WRITE);

	if (res != FR_OK) {
		Error_Handler();
	}

	//Write to the text file
	size_t sz = snprintf((char*) write_data, sizeof(write_data), "%.0lu,%.3f,%.3f,%.3f\n", time_uS, altitude, pressure, temperature);

	res = f_write(&SDFile, write_data, sz, (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		Error_Handler();
	} else {

		f_close(&SDFile);
	}
	return res;
}

FRESULT SD_write_GPS_data(uint32_t time_uS, int time_hours, int time_minutes, int time_seconds, int date_day, int date_month, int date_year, int hour_offset, int minute_offset, float latitude, float longitude, float altitude, int fix_quality, int satelites_tracked) {
	uint8_t write_data[_MAX_SS];
	uint32_t byteswritten;
	char dateTime[100];
	char gps_fname[32];
	sprintf(gps_fname, "%s/%s", directory_name, GPSDir);
	FRESULT res = f_open(&SDFile, gps_fname, FA_OPEN_APPEND | FA_WRITE);

	if (res != FR_OK) {
		Error_Handler();
	}

	snprintf(dateTime, sizeof(dateTime), "%d:%d:%d %02d.%02d.%d UTC%+03d:%02d", time_hours, time_minutes, time_seconds, date_day, date_month, date_year, hour_offset, minute_offset);
	char LLA_Sat_Fix_Qual[100];
	snprintf(LLA_Sat_Fix_Qual, sizeof(LLA_Sat_Fix_Qual), "%f, %f, %f, %d, %d", latitude, longitude, altitude, fix_quality, satelites_tracked);
	size_t sz = snprintf((char*) write_data, sizeof(write_data), "%.0lu,%s,%s\n", time_uS, LLA_Sat_Fix_Qual, dateTime);

	res = f_write(&SDFile, write_data, sz, (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		Error_Handler();
	} else {

		f_close(&SDFile);
	}
	return res;
}

FRESULT SD_write_system_state_data(uint32_t time_uS, uint8_t tranmsitting_gps, int flight_state, float starting_altitude, uint32_t launch_time, uint8_t controller_saturated) {
	uint8_t write_data[_MAX_SS];
	uint32_t byteswritten;
	char sys_fname[32];
	sprintf(sys_fname, "%s/%s", directory_name, systemStateDir);
	FRESULT res = f_open(&SDFile, sys_fname, FA_OPEN_APPEND | FA_WRITE);

	if (res != FR_OK) {
		Error_Handler();
	}

	//Write to the text file
	size_t sz = snprintf((char*) write_data, sizeof(write_data), "%.0lu,%d,%d,%f,%.0lu,%d\n", time_uS, tranmsitting_gps, flight_state, starting_altitude, launch_time, controller_saturated);
	res = f_write(&SDFile, write_data, sz, (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		Error_Handler();
	} else {

		f_close(&SDFile);
	}
	return res;
}

FRESULT SD_write_ekf_data(uint32_t time_uS, float qu1, float qu2, float qu3, float qu4) {
	uint8_t write_data[_MAX_SS];
	uint32_t byteswritten;
	char sys_fname[32];
	sprintf(sys_fname, "%s/%s", directory_name, ekfDir);
	FRESULT res = f_open(&SDFile, sys_fname, FA_OPEN_APPEND | FA_WRITE);

	if (res != FR_OK) {
		Error_Handler();
	}

	//Write to the text file
	size_t sz = snprintf((char*) write_data, sizeof(write_data), "%.0lu,%f,%f,%f,%f\n", time_uS, qu1, qu2, qu3, qu4);
	res = f_write(&SDFile, write_data, sz, (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		Error_Handler();
	} else {

		f_close(&SDFile);
	}
	return res;
}

FRESULT SD_write_binary_stream(uint8_t *buffer, size_t len) {
	// Buffer is written with header to be able to find the start of each packet in the data file
	uint8_t write_data[_MAX_SS];
	uint32_t byteswritten;
	char stream_fname[32];
	sprintf(stream_fname, "%s/%s", directory_name, streamDir);
	FRESULT res = f_open(&SDFile, stream_fname, FA_OPEN_APPEND | FA_WRITE);

	if (res != FR_OK) {
		Error_Handler();
	}

	//Write to the text file
	res = f_write(&SDFile, buffer, len, (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		Error_Handler();
	} else {

		f_close(&SDFile);
	}
	return res;
}
