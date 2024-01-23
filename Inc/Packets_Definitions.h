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
#define STREAM_PACKET_CONFIG_SET   0x23
#define STREAM_PACKET_TYPE_0    0x24
#define STREAM_PACKET_TYPE_1    0x25
#define STREAM_PACKET_TYPE_2    0x26
#define STREAM_PACKET_TYPE_3    0x27
#define STREAM_PACKET_TYPE_4    0x28
#define STREAM_PACKET_TYPE_5    0x29
#define STREAM_PACKET_TYPE_6    0x2A
#define STREAM_PACKET_TYPE_7    0x2B
#define STREAM_PACKET_CONFIG_REQ	0x2C
#define STREAM_PACKET_CONFIG_RES	0x2D
#define HEART_BEAT_CONFIG_PACKET_SET	0x2E
#define HEART_BEAT_PACKET		0x2F
#define ARM_DROGUE_REQ			0x30
#define ARM_MAIN_REQ			0x31
#define ARM_MAIN_RES			0x32
#define ARM_DROGUE_RES			0x33
#define SYSTEM_STATE_PACKET_REQ	0x34
#define SYSTEM_STATE_PACKET_TYPE_0_RES		0x35
#define SYSTEM_STATE_PACKET_TYPE_1_RES		0x36
#define SYSTEM_STATE_PACKET_TYPE_2_RES		0x37
#define SYSTEM_STATE_PACKET_TYPE_3_RES		0x38
#define SYSTEM_STATE_PACKET_TYPE_4_RES		0x39
#define SYSTEM_STATE_PACKET_TYPE_5_RES		0x3A
#define SYSTEM_STATE_PACKET_TYPE_6_RES		0x3B
#define SYSTEM_STATE_PACKET_TYPE_7_RES		0x3C


/* Payload lengths */
#define BAT_VOL_REQ_PACKET_LEN 0x00
#define BAT_VOL_RES_PACKET_LEN 0x04
#define CONTINUITY_REQ_PACKET_LEN 0x00
#define CONTINUITY _RES_PACKET_LEN 0x02
#define FIRE_DROGUE_REQ_PACKET_LEN 0x00
#define FIRE_DROGUE_RES_PACKET_LEN 0x01
#define FIRE_MAIN_REQ_PACKET_LEN 0x00
#define FIRE_MAIN_RES_PACKET_LEN 0x01
#define GPS1_STATE_REQ_PACKET_LEN 0x00
#define GPS1_STATE_RES_PACKET_LEN 0x0E
#define GPS2_STATE_REQ_PACKET_LEN 0x00
#define GPS2_STATE_RES_PACKET_LEN 0x0E
#define ACCEL1_STATE_REQ_PACKET_LEN 0x00
#define ACCEL1_STATE_RES_PACKET_LEN 0x0D
#define ACCEL2_STATE_REQ_PACKET_LEN 0x00
#define ACCEL2_STATE_RES_PACKET_LEN 0x0D
#define GYRO1_STATE_REQ_PACKET_LEN 0x00
#define GYRO1_STATE_RES_PACKET_LEN 0x0D
#define GYRO2_STATE_REQ_PACKET_LEN 0x00
#define GYRO2_STATE_RES_PACKET_LEN 0x0D
#define MAG1_STATE_REQ_PACKET_LEN 0x00
#define MAG1_STATE_RES_PACKET_LEN 0x0D
#define MAG2_STATE_REQ_PACKET_LEN 0x00
#define MAG2_STATE_RES_PACKET_LEN 0x0D
#define BARO1_STATE_REQ_PACKET_LEN 0x00
#define BARO1_STATE_RES_PACKET_LEN 0x0D
#define BARO2_STATE_REQ_PACKET_LEN 0x00
#define BARO2_STATE_RES_PACKET_LEN 0x0D
#define FLASH_MEMORY_STATE_REQ_PACKET_LEN 0x00
#define FLASH_MEMORY_STATE_RES_PACKET_LEN 0x05
#define FLASH_MEMORY_CONFIG_SET_PACKET_LEN 0x00
#define GPS_TRACKING_CONFIG_RES_PACKET_LEN 0x05
#define GPS_TRACKING_CONFIG_SET_PACKET_LEN 0x05
#define GPS_TRACKING_PACKET_PACKET_LEN 0x0D
#define STREAM_PACKET_CONFIG_LEN 0x28
#define STREAM_PACKET_TYPE_0_LEN 0x4E
#define STREAM_PACKET_TYPE_1_LEN 0x00
#define STREAM_PACKET_TYPE_2_LEN 0x00
#define STREAM_PACKET_TYPE_3_LEN 0x00
#define STREAM_PACKET_TYPE_4_LEN 0x00
#define STREAM_PACKET_TYPE_5_LEN 0x00
#define STREAM_PACKET_TYPE_6_LEN 0x00
#define STREAM_PACKET_TYPE_7_LEN 0x00
#define STREAM_PACKET_CONFIG_REQ_LEN 0x00
#define STREAM_PACKET_CONFIG_RES_LEN 0x05
#define HEART_BEAT_CONFIG_PACKET_SET_LEN 0x05
#define HEART_BEAT_PACKET_LEN 	0x00
#define ARM_DROGUE_REQ_LEN 		0x01
#define ARM_MAIN_REQ_LEN 		0x01
#define ARM_MAIN_RES_LEN 		0x01
#define ARM_DROGUE_RES_LEN 		0x01
#define SYSTEM_STATE_PACKET_REQ_LEN 0x01
#define SYSTEM_STATE_PACKET_TYPE_0_RES_LEN 0x80
#define SYSTEM_STATE_PACKET_TYPE_1_RES_LEN 0x00
#define SYSTEM_STATE_PACKET_TYPE_2_RES_LEN 0x00
#define SYSTEM_STATE_PACKET_TYPE_3_RES_LEN 0x00
#define SYSTEM_STATE_PACKET_TYPE_4_RES_LEN 0x00
#define SYSTEM_STATE_PACKET_TYPE_5_RES_LEN 0x00
#define SYSTEM_STATE_PACKET_TYPE_6_RES_LEN 0x00
#define SYSTEM_STATE_PACKET_TYPE_7_RES_LEN 0x00

typedef struct __attribute__((__packed__))
{
	float battery_voltage;
} bat_vol_res;

typedef struct __attribute__((__packed__))
{
	uint8_t drogue_ematch_state;
	uint8_t main_ematch_state;
} continuity_res;

typedef struct __attribute__((__packed__))
{
	uint8_t fire_drogue_result;
} fire_drogue_res;

typedef struct
{
	uint8_t fire_main_result;
} fire_main_res;

typedef struct __attribute__((__packed__))
{
	uint8_t gps1_good;
	float gps1_latitude;
	float gps1_longitude;
	float gps1_altitude;
	uint8_t gps1_satellites_tracked;
} gps1_state_res;

typedef struct __attribute__((__packed__))
{
	uint8_t gps2_good;
	float gps2_latitude;
	float gps2_longitude;
	float gps2_altitude;
	uint8_t gps2_satellites_tracked;
} gps2_state_res;

typedef struct __attribute__((__packed__))
{
	uint8_t acc1_good;
	float acc1X;
	float acc1Y;
	float acc1Z;
} accel1_state_res;

typedef struct __attribute__((__packed__))
{
	uint8_t acc2_good;
	float acc2X;
	float acc2Y;
	float acc2Z;
} accel2_state_res;

typedef struct __attribute__((__packed__))
{
	uint8_t gyro1_good;
	float gyro1X;
	float gyro1Y;
	float gyro1Z;
} gyro1_state_res;

typedef struct __attribute__((__packed__))
{
	uint8_t gyro2_good;
	float gyro2X;
	float gyro2Y;
	float gyro2Z;
} gyro2_state_res;

typedef struct __attribute__((__packed__))
{
	uint8_t mag1_good;
	float mag1X;
	float mag1Y;
	float mag1Z;
} mag1_state_res;

typedef struct __attribute__((__packed__))
{
	uint8_t mag2_good;
	float mag2X;
	float mag2Y;
	float mag2Z;
} mag2_state_res;

typedef struct __attribute__((__packed__))
{
	uint8_t baro1_good;
	float baro1_pressure;
	float baro1_temperature;
	float baro1_altitude;
} baro1_state_res;

typedef struct __attribute__((__packed__))
{
	uint8_t baro2_good;
	float baro2_pressure;
	float baro2_temperature;
	float baro2_altitude;
} baro2_state_res;

typedef struct __attribute__((__packed__))
{
	uint8_t flash_good;
	float flash_write_speed;
	float available_flash_memory;
} flash_state_res;

typedef struct __attribute__((__packed__))
{
	uint8_t flash_logging_enabled
	float flash_write_speed;
} flash_memory_config_set;

typedef struct __attribute__((__packed__))
{
	uint8_t gps_tracking_enabled;
	float gps_tracking_chirp_frequency;
} gps_tracking_config_res;

typedef struct __attribute__((__packed__))
{
	uint8_t gps_tracking_enabled;
	float gps_tracking_chirp_frequency;
} gps_tracking_config_set;

typedef struct __attribute__((__packed__))
{
	float gps1_latitude;
	float gps1_longitude;
	float gps1_altitude;
	uint8_t gps1_satellites_tracked;
} gps_tracking_packet;

typedef struct __attribute__((__packed__))
{
	uint8_t stream_packet_type_enabled;
	float packet_stream_frequency;
} stream_packet_config_set;

typedef struct __attribute__((__packed__))
{
	uint32_t timestamp;
	float gps1_latitude;
	float gps1_longitude;
	float gps1_altitude;
	uint8_t gps1_satellites_tracked;
	float acc1X;
	float acc1Y;
	float acc1Z;
	float velX;
	float velY;
	float velZ;
	float baro1_altitude;
	float gyro1X;
	float gyro1Y;
	float gyro1Z;
	float quaternion_q1;
	float quaternion_q2;
	float quaternion_q3;
	float quaternion_q4;
	float battery_voltage;
	uint8_t flight_state;
	float ambient_temperature;
	float available_flash_memory;
} stream_packet_type_0;

typedef struct __attribute__((__packed__)) 
{

} stream_packet_type_1;

typedef struct __attribute__((__packed__)) 
{

} stream_packet_type_2;

typedef struct __attribute__((__packed__)) 
{

} stream_packet_type_3;

typedef struct __attribute__((__packed__)) 
{

} stream_packet_type_4;

typedef struct __attribute__((__packed__)) 
{

} stream_packet_type_5;

typedef struct __attribute__((__packed__)) 
{

} stream_packet_type_6;

typedef struct __attribute__((__packed__)) 
{

} stream_packet_type_7;

typedef struct __attribute__((__packed__)) 
{
	uint8_t stream_packet_type_enabled;
	float packet_stream_frequency;
} stream_packet_config_res;

typedef struct __attribute__((__packed__)) 
{
	uint8_t heart_beat_enabled;
	float heart_beat_chirp_frequency;
} heart_beat_config_packet_set;

typedef struct __attribute__((__packed__)) 
{

} heart_beat_packet;

typedef struct __attribute__((__packed__)) 
{
	uint8_t drogue_arm_state_set;
} arm_drogue_req;

typedef struct __attribute__((__packed__)) 
{
	uint8_t main_arm_state_set;
} arm_main_req;

typedef struct __attribute__((__packed__)) 
{
	uint8_t arm_main_state;
} arm_main_res;

typedef struct __attribute__((__packed__)) 
{
	uint8_t arm_drogue_state;
} arm_drogue_res;

typedef struct __attribute__((__packed__)) 
{
	uint8_t state_packet_type;
} system_state_packet_req;

typedef struct __attribute__((__packed__)) 
{
	uint32_t timestamp;
	uint32_t battery_voltage;
	uint8_t drogue_ematch_state;
	uint8_t main_ematch_state;
	uint8_t arm_drogue_state;
	uint8_t arm_main_state;
	uint8_t gps1_good;
	float gps1_latitude;
	float gps1_longitude;
	float gps1_satellites_tracked;
	uint8_t acc1_good;
	float acc1X;
	float acc1Y;
	float acc1Z;
	uint8_t acc2_good;
	float acc2X;
	float acc2Y;
	float acc2Z
	uint8_t gyro1_good;
	float gyro1X;
	float gyro1Y;
	float gyro1Z;
	uint8_t gyro2_good;
	float gyro2X;
	float gyro2Y;
	float gyro2Z;
	uint8_t mag1_good;
	float mag1X;
	float mag1Y;
	float mag1Z;
	uint8_t baro1_good;
	float baro1_pressure;
	float baro1_temperature;
	float baro1_altitude;
	uint8_t flash_good;
	float flash_write_speed;
	float available_flash_memory;
	uint8_t gps_tracking_enabled;
	float gps_tracking_chirp_frequency;
	uint8_t stream_packet_type_enabled;
	float packet_stream_frequency;
	uint8_t heart_beat_enabled;
	float heart_beat_chirp_frequency;
} system_state_packet_type_0_res;

typedef struct __attribute__((__packed__)) 
{
} stream_state_packet_type_1_res;

typedef struct __attribute__((__packed__)) 
{
} stream_state_packet_type_2_res;

typedef struct __attribute__((__packed__)) 
{
} stream_state_packet_type_3_res;

typedef struct __attribute__((__packed__)) 
{
} stream_state_packet_type_4_res;

typedef struct __attribute__((__packed__)) 
{
} stream_state_packet_type_5_res;

typedef struct __attribute__((__packed__)) 
{
} stream_state_packet_type_6_res;

typedef struct __attribute__((__packed__)) 
{
} stream_state_packet_type_7_res;

#endif /* INC_PACKETS_H_ */
