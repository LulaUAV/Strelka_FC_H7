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
#define BAT_VOL_REQ             0x00
#define BAT_VOL_RES             0x01
#define CONTINUITY_REQ          0x02
#define CONTINUITY_RES          0x03
#define FIRE_DROGUE_REQ         0x04
#define FIRE_DROGUE_RES         0x05
#define FIRE_MAIN_REQ           0x06
#define FIRE_MAIN_RES           0x07
#define GPS1_STATE_REQ          0x08
#define GPS1_STATE_RES          0x09
#define GPS2_STATE_REQ          0x0A
#define GPS2_STATE_RES          0x0B
#define ACCEL1_STATE_REQ        0x0C
#define ACCEL1_STATE_RES        0x0D
#define ACCEL2_STATE_REQ        0x0E
#define ACCEL2_STATE_RES        0x0F
#define GYRO1_STATE_REQ         0x10
#define GYRO1_STATE_RES         0x11
#define GYRO2_STATE_REQ         0x12
#define GYRO2_STATE_RES         0x13
#define MAG1_STATE_REQ          0x14
#define MAG1_STATE_RES          0x15
#define MAG2_STATE_REQ          0x16
#define MAG2_STATE_RES          0x17
#define BARO1_STATE_REQ         0x18
#define BARO1_STATE_RES         0x19
#define BARO2_STATE_REQ         0x1A
#define BARO2_STATE_RES         0x1B
#define FLASH_MEMORY_STATE_REQ  0x1C
#define FLASH_MEMORY_STATE_RES  0x1D
#define FLASH_MEMORY_CONFIG_SET 0x1E
#define GPS_TRACKING_CONFIG_REQ 0x1F
#define GPS_TRACKING_CONFIG_RES 0x20
#define GPS_TRACKING_CONFIG_SET 0x21
#define GPS_TRACKING_PACKET     0x22
#define STREAM_PKT_CONFIG_SET   0x23
#define STREAM_PACKET_TYPE_0    0x24
#define STREAM_PACKET_TYPE_1    0x25
#define STREAM_PACKET_TYPE_2    0x26
#define STREAM_PACKET_TYPE_3    0x27
#define STREAM_PACKET_TYPE_4    0x28
#define STREAM_PACKET_TYPE_5    0x29
#define STREAM_PACKET_TYPE_6    0x2A
#define STREAM_PACKET_TYPE_7    0x2B


/* Payload lengths */
#define BAT_VOL_REQ_PKT_LEN 0x00
#define BAT_VOL_RES_PKT_LEN 0x04
#define CONTINUITY_REQ_PKT_LEN 0x00
#define CONTINUITY _RES_PKT_LEN 0x02
#define FIRE_DROGUE_REQ_PKT_LEN 0x00
#define FIRE_DROGUE_RES_PKT_LEN 0x01
#define FIRE_MAIN_REQ_PKT_LEN 0x00
#define FIRE_MAIN_RES_PKT_LEN 0x01
#define GPS1_STATE_REQ_PKT_LEN 0x00
#define GPS1_STATE_RES_PKT_LEN 0x0E
#define GPS2_STATE_REQ_PKT_LEN 0x00
#define GPS2_STATE_RES_PKT_LEN 0x0E
#define ACCEL1_STATE_REQ_PKT_LEN 0x00
#define ACCEL1_STATE_RES_PKT_LEN 0x0D
#define ACCEL2_STATE_REQ_PKT_LEN 0x00
#define ACCEL2_STATE_RES_PKT_LEN 0x0D
#define GYRO1_STATE_REQ_PKT_LEN 0x00
#define GYRO1_STATE_RES_PKT_LEN 0x0D
#define GYRO2_STATE_REQ_PKT_LEN 0x00
#define GYRO2_STATE_RES_PKT_LEN 0x0D
#define MAG1_STATE_REQ_PKT_LEN 0x00
#define MAG1_STATE_RES_PKT_LEN 0x0D
#define MAG2_STATE_REQ_PKT_LEN 0x00
#define MAG2_STATE_RES_PKT_LEN 0x0D
#define BARO1_STATE_REQ_PKT_LEN 0x00
#define BARO1_STATE_RES_PKT_LEN 0x0D
#define BARO2_STATE_REQ_PKT_LEN 0x00
#define BARO2_STATE_RES_PKT_LEN 0x0D
#define FLASH_MEMORY_STATE_REQ_PKT_LEN 0x00
#define FLASH_MEMORY_STATE_RES_PKT_LEN 0x05
#define FLASH_MEMORY_CONFIG_SET_PKT_LEN 0x00
#define GPS_TRACKING_CONFIG_RES_PKT_LEN 0x05
#define GPS_TRACKING_CONFIG_SET_PKT_LEN 0x04
#define GPS_TRACKING_PACKET_PKT_LEN 0x0D
#define STREAM_PKT_CONFIG_LEN 0x28
#define STREAM_PACKET_TYPE_0_LEN 0x4E
#define STREAM_PACKET_TYPE_1_LEN 0x00
#define STREAM_PACKET_TYPE_2_LEN 0x00
#define STREAM_PACKET_TYPE_3_LEN 0x00
#define STREAM_PACKET_TYPE_4_LEN 0x00
#define STREAM_PACKET_TYPE_5_LEN 0x00
#define STREAM_PACKET_TYPE_6_LEN 0x00
#define STREAM_PACKET_TYPE_7_LEN 0x00

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

typedef struct 
{
	uint8_t packet_type_0_enable;
	float packet_type_0_stream_frequency;
	uint8_t packet_type_1_enable;
	float packet_type_1_stream_frequency;
	uint8_t packet_type_2_enable;
	float packet_type_2_stream_frequency;
	uint8_t packet_type_3_enable;
	float packet_type_3_stream_frequency;
	uint8_t packet_type_4_enable;
	float packet_type_4_stream_frequency;
	uint8_t packet_type_5_enable;
	float packet_type_5_stream_frequency;
	uint8_t packet_type_6_enable;
	float packet_type_6_stream_frequency;
	uint8_t packet_type_7_enable;
	float packet_type_7_stream_frequency;
} stream_packet_config_set;

typedef struct 
{
	uint32_t timestamp;
	float latitude;
	float longitude;
	float gps_altitude;
	uint8_t satellites_tracked;
	float lin_accX;
	float lin_accY;
	float lin_accZ;
	float lin_velX;
	float lin_velY;
	float lin_velZ;
	float baro_altitude;
	float ang_velX;
	float ang_velY;
	float ang_velZ;
	float quaternion_q1;
	float quaternion_q2;
	float quaternion_q3;
	float quaternion_q4;
	float battery_voltage;
	uint8_t flight_state;
	float ambient_temperature;
	float available_flash_kb;
} stream_packet_type_0;

typedef struct 
{

} stream_packet_type_1;

typedef struct 
{

} stream_packet_type_2;

typedef struct 
{

} stream_packet_type_3;

typedef struct 
{

} stream_packet_type_4;

typedef struct 
{

} stream_packet_type_5;

typedef struct 
{

} stream_packet_type_6;

typedef struct 
{

} stream_packet_type_7;

#endif /* INC_PACKETS_H_ */
