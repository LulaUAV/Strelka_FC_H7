/*
 * Packet_Definitions.h
 *
 *  Created on: 17 Dec. 2023
 *      Author: Angus McLennan
 */

#ifndef INC_PACKETS_H_
#define INC_PACKETS_H_

#include <stdint.h>

/* Packet identifier definitions */
#define BAT_VOL_REQ
#define BAT_VOL_RES
#define CONTINUITY_REQ
#define CONTINUITY _RES
#define FIRE_DROGUE_REQ
#define FIRE_DROGUE_RES
#define FIRE_MAIN_REQ
#define FIRE_MAIN_RES
#define GPS1_STATE_REQ
#define GPS1_STATE_RES
#define GPS2_STATE_REQ
#define GPS2_STATE_RES
#define ACCEL1_STATE_REQ
#define ACCEL1_STATE_RES
#define ACCEL2_STATE_REQ
#define ACCEL2_STATE_RES
#define GYRO1_STATE_REQ
#define GYRO1_STATE_RES
#define GYRO2_STATE_REQ
#define GYRO2_STATE_RES
#define MAG1_STATE_REQ
#define MAG1_STATE_RES
#define MAG2_STATE_REQ
#define MAG2_STATE_RES
#define BARO1_STATE_REQ
#define BARO1_STATE_RES
#define BARO2_STATE_REQ
#define BARO2_STATE_RES
#define FLASH_MEMORY_STATE_REQ
#define FLASH_MEMORY_STATE_RES
#define FLASH_MEMORY_CONFIG_SET
#define GPS_TRACKING_CONFIG_RES
#define GPS_TRACKING_CONFIG_SET
#define GPS_TRACKING_PACKET

/* Payload lengths */
#define BAT_VOL_REQ_PKT_LEN
#define BAT_VOL_RES_PKT_LEN
#define CONTINUITY_REQ_PKT_LEN
#define CONTINUITY _RES_PKT_LEN
#define FIRE_DROGUE_REQ_PKT_LEN
#define FIRE_DROGUE_RES_PKT_LEN
#define FIRE_MAIN_REQ_PKT_LEN
#define FIRE_MAIN_RES_PKT_LEN
#define GPS1_STATE_REQ_PKT_LEN
#define GPS1_STATE_RES_PKT_LEN
#define GPS2_STATE_REQ_PKT_LEN
#define GPS2_STATE_RES_PKT_LEN
#define ACCEL1_STATE_REQ_PKT_LEN
#define ACCEL1_STATE_RES_PKT_LEN
#define ACCEL2_STATE_REQ_PKT_LEN
#define ACCEL2_STATE_RES_PKT_LEN
#define GYRO1_STATE_REQ_PKT_LEN
#define GYRO1_STATE_RES_PKT_LEN
#define GYRO2_STATE_REQ_PKT_LEN
#define GYRO2_STATE_RES_PKT_LEN
#define MAG1_STATE_REQ_PKT_LEN
#define MAG1_STATE_RES_PKT_LEN
#define MAG2_STATE_REQ_PKT_LEN
#define MAG2_STATE_RES_PKT_LEN
#define BARO1_STATE_REQ_PKT_LEN
#define BARO1_STATE_RES_PKT_LEN
#define BARO2_STATE_REQ_PKT_LEN
#define BARO2_STATE_RES_PKT_LEN
#define FLASH_MEMORY_STATE_REQ_PKT_LEN
#define FLASH_MEMORY_STATE_RES_PKT_LEN
#define FLASH_MEMORY_CONFIG_SET_PKT_LEN
#define GPS_TRACKING_CONFIG_RES_PKT_LEN
#define GPS_TRACKING_CONFIG_SET_PKT_LEN
#define GPS_TRACKING_PACKET_PKT_LEN

typedef struct
{
	float battery_voltage;
} bat_vol_res;

typedef struct
{
	uint8_t drogue_ematch_state;
	uint8_t main_ematch_state;
} continuity_res;

typedef struct
{
	uint8_t result;
} fire_drogue_res_res;

typedef struct
{
	uint8_t result;
} fire_main_res;

typedef struct
{
	uint8_t gps_good;
	float latitude;
	float longitude;
	float altitude;
	uint8_t satellites_tracked;
} gps1_state_res;

typedef struct
{
	uint8_t gps_good;
	float latitude;
	float longitude;
	float altitude;
	uint8_t satellites_tracked;
} gps2_state_res;

typedef struct
{
	uint8_t acc_good;
	float accX;
	float accY;
	float accZ;
} accel1_state_res;

typedef struct
{
	uint8_t acc_good;
	float accX;
	float accY;
	float accZ;
} accel2_state_res;

typedef struct
{
	uint8_t gyro_good;
	float gyroX;
	float gyroy;
	float gyroz;
} gyro1_state_res;

typedef struct
{
	uint8_t gyro_good;
	float gyroX;
	float gyroy;
	float gyroz;
} gyro2_state_res;

typedef struct
{
	uint8_t mag_good;
	float magX;
	float magY;
	float magZ;
} mag1_state_res;

typedef struct
{
	uint8_t mag_good;
	float magX;
	float magY;
	float magZ;
} mag2_state_res;

typedef struct
{
	uint8_t baro_good;
	float pressure;
	float temperature;
	float altitude;
} baro1_state_res;

typedef struct
{
	uint8_t baro_good;
	float pressure;
	float temperature;
	float altitude;
} baro2_state_res;

typedef struct
{
	uint8_t flash_good;
	float write_speed;
} flash_state_res;

typedef struct
{
	uint8_t gps_good;
	float chirp_frequency;
} gps_tracking_config_res;

typedef struct
{
	float chirp_frequency;
} gps_tracking_config_set;

typedef struct
{
	float latitude;
	float longitude;
	float altitude;
	uint8_t satellites_tracked;
} gps_tracking_packet;

#endif /* INC_PACKETS_H_ */
