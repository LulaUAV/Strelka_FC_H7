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
char internal_smDir[] = "int_sm.csv";
const TickType_t xFrequency;

FRESULT SD_init() {
	/* Flash memory file system config */
	uint8_t rtext[_MAX_SS];/* File read buffer */
	FRESULT res = f_mount(&SDFatFS, (TCHAR const*) SDPath, 0);
	if (res != FR_OK) {
		if (res == FR_NO_FILESYSTEM) {
			res = f_mkfs((TCHAR const*) SDPath, FM_ANY, 0, rtext, sizeof(rtext));
			if (res != FR_OK) {
				if (res == FR_LOCKED) {
					return res;
				}
				Non_Blocking_Error_Handler();
			}
		} else
			Non_Blocking_Error_Handler();
	}

	res = SD_mk_root_dir();
	if (res != FR_OK) {
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
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

FRESULT SD_write_headers() {
	char fname[32];
	uint32_t byteswritten;
	sprintf(fname, "%s/%s", directory_name, accelDir);
	FRESULT res = f_open(&SDFile, fname, FA_OPEN_APPEND | FA_WRITE);
	osDelay(10);
	if (res != FR_OK) {
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
	}
	uint8_t accel_header[] = "timestamp(uS),acc1X(g),acc1Y(g),acc1Z(g),acc2X(g),acc2Y(g),acc2Z(g)\n";
	res = f_write(&SDFile, accel_header, sizeof(accel_header), (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
	} else {

		f_close(&SDFile);
	}
	osDelay(10);

	sprintf(fname, "%s/%s", directory_name, gyroDir);
	res = f_open(&SDFile, fname, FA_OPEN_APPEND | FA_WRITE);
	osDelay(10);
	if (res != FR_OK) {
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
	}
	char gyro_header[] = "timestamp(uS),gyro1X(rad/s),gyro1Y(rad/s),gyro1Z(rad/s),gyro2X(rad/s),gyro2Y(rad/s),gyro2Z(rad/s)\n";
	res = f_write(&SDFile, gyro_header, sizeof(gyro_header), (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
	} else {

		f_close(&SDFile);
	}
	osDelay(10);

	sprintf(fname, "%s/%s", directory_name, magDir);
	res = f_open(&SDFile, fname, FA_OPEN_APPEND | FA_WRITE);
	osDelay(10);
	if (res != FR_OK) {
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
	}
	char mag_header[] = "timestamp(uS),magX(uT),magY(uT),magZ(uT)\n";
	res = f_write(&SDFile, mag_header, sizeof(mag_header), (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
	} else {

		f_close(&SDFile);
	}
	osDelay(10);

	sprintf(fname, "%s/%s", directory_name, baroDir);
	res = f_open(&SDFile, fname, FA_OPEN_APPEND | FA_WRITE);
	osDelay(10);
	if (res != FR_OK) {
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
	}
	char baro_header[] = "timestamp(uS),altitude(m),pressure(Pa),temperature(degC)\n";
	res = f_write(&SDFile, baro_header, sizeof(baro_header), (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
	} else {

		f_close(&SDFile);
	}
	osDelay(10);

	sprintf(fname, "%s/%s", directory_name, systemStateDir);
	res = f_open(&SDFile, fname, FA_OPEN_APPEND | FA_WRITE);
	osDelay(10);
	if (res != FR_OK) {
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
	}
	char sys_header[] = "timestamp(uS), flight state, drogue ematch status, main ematch status, launch time (mS), Launch altitude (m), Burnout time (ms), Burnout altitude (m), drogue deploy time (ms), drogue deploy altitude(m), main deploy time(ms), main deploy altitude(m), landing time(ms), landing altitude(m), battery voltage (V)\n";
	res = f_write(&SDFile, sys_header, sizeof(sys_header), (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
	} else {

		f_close(&SDFile);
	}
	osDelay(10);

	sprintf(fname, "%s/%s", directory_name, ekfDir);
	res = f_open(&SDFile, fname, FA_OPEN_APPEND | FA_WRITE);
	osDelay(10);
	if (res != FR_OK) {
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
	}
	char ekf_header[] = "timstamp (us), q0(w), q1(x), q2(y), q3(z), update enabled\n";
	res = f_write(&SDFile, ekf_header, sizeof(ekf_header), (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
	} else {

		f_close(&SDFile);
	}
	osDelay(10);

	sprintf(fname, "%s/%s", directory_name, internal_smDir);
	res = f_open(&SDFile, fname, FA_OPEN_APPEND | FA_WRITE);
	osDelay(10);
	if (res != FR_OK) {
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
	}

	char internal_sm_header[] = "Timestamp (uS), Angle from vertical (rad), Filtered launch detection acceleration (g), Filtered burnout detection x acceleration (g), Filtered apogee detection altitude (m), Filtered apogee detection vertical velocity (m/s), Filtered apogee detection acceleration (g), Unfiltered main detection agl altitude (m), Filtered landing detection vertical velocity (m), Up axis index\n";
	res = f_write(&SDFile, internal_sm_header, sizeof(internal_sm_header), (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
	} else {

		f_close(&SDFile);
	}
	osDelay(10);
	return res;
}

FRESULT SD_write_accelerometer_data(uint32_t time_uS, float acc1X, float acc1Y, float acc1Z, float acc2X, float acc2Y, float acc2Z) {
	char accel_fname[32];
	uint8_t write_data[_MAX_SS];
	uint32_t byteswritten;
	sprintf(accel_fname, "%s/%s", directory_name, accelDir);
	FRESULT res = f_open(&SDFile, accel_fname, FA_OPEN_APPEND | FA_WRITE);

	if (res != FR_OK) {
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
	}

	//Write to the text file
	size_t sz = snprintf((char*) write_data, sizeof(write_data), "%.0lu,%.3f,%.3f,%.3,%.3f,%.3f,%.3f\n", time_uS, acc1X, acc1Y, acc1Z, acc2X, acc2Y, acc2Z);

	res = f_write(&SDFile, write_data, sz, (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
	} else {

		f_close(&SDFile);
	}
	return res;
}

FRESULT SD_write_gyroscope_data(uint32_t time_uS, float gyro1X, float gyro1Y, float gyro1Z, float gyro2X, float gyro2Y, float gyro2Z) {
	uint8_t write_data[_MAX_SS];
	uint32_t byteswritten;
	char gyro_fname[32];
	sprintf(gyro_fname, "%s/%s", directory_name, gyroDir);
	FRESULT res = f_open(&SDFile, gyro_fname,
	FA_OPEN_APPEND | FA_WRITE);

	if (res != FR_OK) {
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
	}

	//Write to the text file
	size_t sz = snprintf((char*) write_data, sizeof(write_data), "%.0lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", time_uS, gyro1X, gyro1Y, gyro1Z, gyro2X, gyro2Y, gyro2Z);

	res = f_write(&SDFile, write_data, sz, (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
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
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
	}

	//Write to the text file
	size_t sz = snprintf((char*) write_data, sizeof(write_data), "%.0lu,%.3f,%.3f,%.3f\n", time_uS, magX, magY, magZ);

	res = f_write(&SDFile, write_data, sz, (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
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
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
	}

	//Write to the text file
	size_t sz = snprintf((char*) write_data, sizeof(write_data), "%.0lu,%.3f,%.3f,%.3f\n", time_uS, altitude, pressure, temperature);

	res = f_write(&SDFile, write_data, sz, (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		if (res == FR_LOCKED) {
			return res;
		}
		Non_Blocking_Error_Handler();
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
		return res;
	}

	snprintf(dateTime, sizeof(dateTime), "%d:%d:%d %02d.%02d.%d UTC%+03d:%02d", time_hours, time_minutes, time_seconds, date_day, date_month, date_year, hour_offset, minute_offset);
	char LLA_Sat_Fix_Qual[100];
	snprintf(LLA_Sat_Fix_Qual, sizeof(LLA_Sat_Fix_Qual), "%f, %f, %f, %d, %d", latitude, longitude, altitude, fix_quality, satelites_tracked);
	size_t sz = snprintf((char*) write_data, sizeof(write_data), "%.0lu,%s,%s\n", time_uS, LLA_Sat_Fix_Qual, dateTime);

	res = f_write(&SDFile, write_data, sz, (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		return res;
	} else {

		f_close(&SDFile);
	}
	return res;
}

FRESULT SD_write_system_state_data(uint32_t time_uS, uint8_t flight_state, uint8_t drogue_ematch_status, uint8_t main_ematch_status, uint32_t launch_time, uint32_t drogue_deploy_time, float drogue_deploy_altitude, uint32_t main_deploy_time, float main_deploy_altitude, uint32_t landing_time, float landing_altitude, float battery_voltage) {
	uint8_t write_data[_MAX_SS];
	uint32_t byteswritten;
	char sys_fname[32];
	sprintf(sys_fname, "%s/%s", directory_name, systemStateDir);
	FRESULT res = f_open(&SDFile, sys_fname, FA_OPEN_APPEND | FA_WRITE);

	if (res != FR_OK) {
		return res;
	}
	//Write to the text file
	size_t sz = snprintf((char*) write_data, sizeof(write_data), "%.0lu,%d,%d,%d,%.0lu,%.0lu,%0.2f,%.0lu,%0.2f,%.0lu,%0.2f,%0.2f,\n", time_uS, flight_state, drogue_ematch_status, main_ematch_status, launch_time, drogue_deploy_time, drogue_deploy_altitude, main_deploy_time, main_deploy_altitude, landing_time, landing_altitude, battery_voltage);
	res = f_write(&SDFile, write_data, sz, (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		return res;
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
		return res;
	}

	//Write to the text file
	size_t sz = snprintf((char*) write_data, sizeof(write_data), "%.0lu,%f,%f,%f,%f\n", time_uS, qu1, qu2, qu3, qu4);
	res = f_write(&SDFile, write_data, sz, (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		return res;
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
		return res;
	}

	//Write to the text file
	res = f_write(&SDFile, buffer, len, (void*) &byteswritten);

	if ((byteswritten == 0) || (res != FR_OK)) {
		return res;
	} else {

		f_close(&SDFile);
	}
	return res;
}

FRESULT SD_get_free_space_kB(float *kBytes_free) {
	DWORD fre_clust, fre_sect;
	FATFS *SDFatFS_ptr_ptr = &SDFatFS;
	// Get free clusters
	FRESULT res = f_getfree((TCHAR const*) SDPath, &fre_clust, &SDFatFS_ptr_ptr);
	// Calculate free sectors
	fre_sect = fre_clust * SDFatFS_ptr_ptr->csize;
	// Assuming 512 byes/sector
	*kBytes_free = ((float) fre_sect) / 2.0;
	return res;
}

FRESULT delete_directory_contents(const TCHAR *path) {
	FRESULT result;
	DIR dir;
	FILINFO fileInfo;

	result = f_opendir(&dir, path);
	if (result != FR_OK) {
		return result;
	}

	while (1) {
		result = f_readdir(&dir, &fileInfo);
		if (result != FR_OK || fileInfo.fname[0] == 0) {
			break;  // End of directory or read error
		}

		if (fileInfo.fname[0] == '.') {
			continue;  // Ignore '.' and '..'
		}

		TCHAR file_path[_MAX_LFN + 1];
		snprintf(file_path, sizeof(file_path), "%s/%s", path, fileInfo.fname);

		if (fileInfo.fattrib & AM_DIR) {
			// It's a directory
			result = delete_directory_contents(file_path);
			if (result != FR_OK) {
				return result;
			}
		} else {
			// It's a file
			result = f_unlink(file_path);
			if (result != FR_OK) {
				return result;
			}
		}
	}

	f_closedir(&dir);
	return FR_OK;
}

FRESULT SD_erase_disk() {
	FATFS fs;
	FRESULT result;

	result = f_mount(&fs, "", 0);  // Mount the default drive
	if (result != FR_OK) {
		if (result == FR_LOCKED) {
			return result;
		}
		return result;  // Handle mount error
	}

	result = delete_directory_contents("/");  // Erase contents starting from root directory
	if (result != FR_OK) {
		f_mount(NULL, "", 0);  // Unmount the default drive on error
		return result;  // Handle delete error
	}

	result = f_mount(NULL, "", 0);  // Unmount the default drive
	return result;  // Return the result of unmount operation
}

void SD_write_accel_batch(uint8_t *accel_buffer, size_t accel_sz) {
	uint32_t byteswritten;
	// Write accel data
	char accel_fname[32];
	sprintf(accel_fname, "%s/%s", directory_name, accelDir);
	FRESULT res = f_open(&SDFile, accel_fname, FA_OPEN_APPEND | FA_WRITE);
	if (res != FR_OK) {
		return res;
	}
	res = f_write(&SDFile, accel_buffer, accel_sz, (void*) &byteswritten);
	f_close(&SDFile);
	if ((byteswritten == 0) || (res != FR_OK)) {
		return res;
	}
}

void SD_write_gyro_batch(uint8_t *gyro_buffer, size_t gyro_sz) {
	uint32_t byteswritten;
	// Write gyro data
	char gyro_fname[32];
	sprintf(gyro_fname, "%s/%s", directory_name, gyroDir);
	FRESULT res = f_open(&SDFile, gyro_fname, FA_OPEN_APPEND | FA_WRITE);
	if (res != FR_OK) {
		return res;
	}
	res = f_write(&SDFile, gyro_buffer, gyro_sz, (void*) &byteswritten);
	f_close(&SDFile);
	if ((byteswritten == 0) || (res != FR_OK)) {
		return res;
	}
}

void SD_write_mag_batch(uint8_t *mag_buffer, size_t mag_sz) {
	uint32_t byteswritten;
	// Write mag data
	char mag_fname[32];
	sprintf(mag_fname, "%s/%s", directory_name, magDir);
	FRESULT res = f_open(&SDFile, mag_fname, FA_OPEN_APPEND | FA_WRITE);
	if (res != FR_OK) {
		return res;
	}
	res = f_write(&SDFile, mag_buffer, mag_sz, (void*) &byteswritten);
	f_close(&SDFile);
	if ((byteswritten == 0) || (res != FR_OK)) {
		return res;
	}
}

void SD_write_baro_batch(uint8_t *baro_buffer, size_t baro_sz) {
	uint32_t byteswritten;
	// Write baro data
	char baro_fname[32];
	sprintf(baro_fname, "%s/%s", directory_name, baroDir);
	FRESULT res = f_open(&SDFile, baro_fname, FA_OPEN_APPEND | FA_WRITE);
	if (res != FR_OK) {
		return res;
	}
	res = f_write(&SDFile, baro_buffer, baro_sz, (void*) &byteswritten);
	f_close(&SDFile);
	if ((byteswritten == 0) || (res != FR_OK)) {
		return res;
	}
}

void SD_write_gps_batch(uint8_t *gps_buffer, size_t gps_sz) {
	uint32_t byteswritten;
	// Write gps data
	char gps_fname[32];
	sprintf(gps_fname, "%s/%s", directory_name, GPSDir);
	FRESULT res = f_open(&SDFile, gps_fname, FA_OPEN_APPEND | FA_WRITE);
	if (res != FR_OK) {
		return res;
	}
	res = f_write(&SDFile, gps_buffer, gps_sz, (void*) &byteswritten);
	f_close(&SDFile);
	if ((byteswritten == 0) || (res != FR_OK)) {
		return res;
	}
}

void SD_write_sys_state_batch(uint8_t *sys_state_buffer, size_t sys_state_sz) {
	uint32_t byteswritten;
	// Write sys_state data
	char sys_fname[32];
	sprintf(sys_fname, "%s/%s", directory_name, systemStateDir);
	FRESULT res = f_open(&SDFile, sys_fname, FA_OPEN_APPEND | FA_WRITE);
	if (res != FR_OK) {
		return res;
	}
	res = f_write(&SDFile, sys_state_buffer, sys_state_sz, (void*) &byteswritten);
	f_close(&SDFile);
	if ((byteswritten == 0) || (res != FR_OK)) {
		return res;
	}
}

void SD_write_ekf_batch(uint8_t *ekf_buffer, size_t ekf_sz) {
	uint32_t byteswritten;
	// Write ekf data
	char ekf_fname[32];
	sprintf(ekf_fname, "%s/%s", directory_name, ekfDir);
	FRESULT res = f_open(&SDFile, ekf_fname, FA_OPEN_APPEND | FA_WRITE);
	if (res != FR_OK) {
		return res;
	}
	res = f_write(&SDFile, ekf_buffer, ekf_sz, (void*) &byteswritten);
	f_close(&SDFile);
	if ((byteswritten == 0) || (res != FR_OK)) {
		return res;
	}
}

void SD_write_internal_state_machine_batch(uint8_t *internal_sm_buffer, size_t internal_sm_sz) {
	uint32_t byteswritten;
	// Write internal state machine data
	char internal_sm_fname[32];
	sprintf(internal_sm_fname, "%s/%s", directory_name, internal_smDir);
	FRESULT res = f_open(&SDFile, internal_sm_fname, FA_OPEN_APPEND | FA_WRITE);
	if (res != FR_OK) {
		return res;
	}
	res = f_write(&SDFile, internal_sm_buffer, internal_sm_sz, (void*) &byteswritten);
	f_close(&SDFile);
	if ((byteswritten == 0) || (res != FR_OK)) {
		return res;
	}
}
