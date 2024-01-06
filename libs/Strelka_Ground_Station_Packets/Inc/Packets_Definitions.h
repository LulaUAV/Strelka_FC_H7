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
#define BAT_VOL_REQ 0x00
#define BAT_VOL_RES 0x01
#define CONTINUITY_REQ 0x02
#define CONTINUITY _RES 0x03
#define FIRE_DROGUE_REQ 0x04
#define FIRE_DROGUE_RES 0x05
#define FIRE_MAIN_REQ 0x06
#define FIRE_MAIN_RES 0x07
#define GPS1_STATE_REQ 0x08
#define GPS1_STATE_RES 0x09
#define GPS2_STATE_REQ 0x0a
#define GPS2_STATE_RES 0x0b
#define ACCEL1_STATE_REQ 0x0c
#define ACCEL1_STATE_RES 0x0d
#define ACCEL2_STATE_REQ 0x0e
#define ACCEL2_STATE_RES 0x0f
#define GYRO1_STATE_REQ 0x10
#define GYRO1_STATE_RES 0x11
#define GYRO2_STATE_REQ 0x12
#define GYRO2_STATE_RES 0x13
#define MAG1_STATE_REQ 0x14
#define MAG1_STATE_RES 0x15
#define MAG2_STATE_REQ 0x16
#define MAG2_STATE_RES 0x17
#define BARO1_STATE_REQ 0x18
#define BARO1_STATE_RES 0x19
#define BARO2_STATE_REQ 0x1a
#define BARO2_STATE_RES 0x1b
#define FLASH_MEMORY_STATE_REQ 0x1c
#define FLASH_MEMORY_STATE_RES 0x1d
#define FLASH_MEMORY_CONFIG_SET 0x1e
#define GPS_TRACKING_CONFIG_REQ 0x1f
#define GPS_TRACKING_CONFIG_RES 0x20
#define GPS_TRACKING_CONFIG_SET 0x21
#define GPS_TRACKING_PACKET 0x22

/* Payload lengths */
#define BAT_VOL_REQ_PKT_LEN 0x00
#define BAT_VOL_RES_PKT_LEN 0x00
#define CONTINUITY_REQ_PKT_LEN 0x00
#define CONTINUITY_RES_PKT_LEN 0x00
#define FIRE_DROGUE_REQ_PKT_LEN 0x00
#define FIRE_DROGUE_RES_PKT_LEN 0x00
#define FIRE_MAIN_REQ_PKT_LEN 0x00
#define FIRE_MAIN_RES_PKT_LEN 0x00
#define GPS1_STATE_REQ_PKT_LEN 0x00
#define GPS1_STATE_RES_PKT_LEN 0x00
#define GPS2_STATE_REQ_PKT_LEN 0x00
#define GPS2_STATE_RES_PKT_LEN 0x00
#define ACCEL1_STATE_REQ_PKT_LEN 0x00
#define ACCEL1_STATE_RES_PKT_LEN 0x00
#define ACCEL2_STATE_REQ_PKT_LEN 0x00
#define ACCEL2_STATE_RES_PKT_LEN 0x00
#define GYRO1_STATE_REQ_PKT_LEN 0x00
#define GYRO1_STATE_RES_PKT_LEN 0x00
#define GYRO2_STATE_REQ_PKT_LEN 0x00
#define GYRO2_STATE_RES_PKT_LEN 0x00
#define MAG1_STATE_REQ_PKT_LEN 0x00
#define MAG1_STATE_RES_PKT_LEN 0x00
#define MAG2_STATE_REQ_PKT_LEN 0x00
#define MAG2_STATE_RES_PKT_LEN 0x00
#define BARO1_STATE_REQ_PKT_LEN 0x00
#define BARO1_STATE_RES_PKT_LEN 0x00
#define BARO2_STATE_REQ_PKT_LEN 0x00
#define BARO2_STATE_RES_PKT_LEN 0x00
#define FLASH_MEMORY_STATE_REQ_PKT_LEN 0x00
#define FLASH_MEMORY_STATE_RES_PKT_LEN 0x00
#define FLASH_MEMORY_CONFIG_SET_PKT_LEN 0x00
#define GPS_TRACKING_CONFIG_RES_PKT_LEN 0x00
#define GPS_TRACKING_CONFIG_SET_PKT_LEN 0x05
#define GPS_TRACKING_PACKET_PKT_LEN 0x00

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
} fire_drogue_res;

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
	float gyroY;
	float gyroZ;
} gyro1_state_res;

typedef struct
{
	uint8_t gyro_good;
	float gyroX;
	float gyroY;
	float gyroZ;
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
	float available_space;
} flash_state_res;

typedef struct
{
	float write_speed;
} flash_memory_config_set;

typedef struct
{
	uint8_t gps_good;
	uint8_t tracking_enabled;
	float chirp_frequency;
} gps_tracking_config_res;

typedef struct __attribute__((__packed__))
{
	uint8_t tracking_enabled;
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
