/*
 * State_Machine.h
 *
 *  Created on: Apr 9, 2023
 *      Author: Angus McLennan
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_
#include <stdbool.h>
//#include "stm32f4xx_hal.h"
//#include "stm32h7xx_hal.h"
#include "main.h"
#include <math.h>
#include <stdlib.h>
#include "arm_math.h"

typedef enum flightState {
	IDLE_ON_PAD, LAUNCHED, BURNOUT, APOGEE, MAIN_CHUTE_ALTITUDE, LANDED,
} flightState;

typedef enum ematchState {
	OPEN_CIRCUIT, SHORT_CIRCUIT, GOOD, EMATCH_ERROR
} ematchState;

typedef enum armState {
	DISARMED, ARMED
} armState;


#ifndef GRAVITY_MPS
#define GRAVITY_MPS							9.81
#endif

// Launch detection constants
#define LAUNCH_ACCEL_FILTER_FREQ			100.0f			// Hz
#define LAUNCH_ACCEL_THRESHOLD 				1.2f			// g
#define PITCH_OVER_ANGLE_THRESHOLD			30.0f			// degrees

// Burnout detection constants
#define BURNOUT_ACCEL_FILTER_FREQ			100.0f			// Hz
#define MAX_MOTOR_BURN_TIME					10.0f			// Seconds
#define BURNOUT_ACCEL_THRESHOLD				-0.5f			// g

// Apogee detection constants
#define VERTICAL_VELOCITY_DETECT_FREQ		100				// Hz
#define VERTICAL_VELOCITY_FILTER_FREQ   	10				// Hz
#define APOGEE_DETECT_VELOCITY_THRESHOLD	1.0f			// m/s

// Main deploy altitude detection constants
#define MAIN_DEPLOY_ALTITUDE				300				// m above the starting altitude (ground)

// Landing detection constants
#define LANDING_SPEED_THRESHOLD				1.0f			// m/s (magnitude)
#define FLIGHT_TIME_TIMOUT					3600			// seconds

#ifndef DEG_TO_RAD
#define DEG_TO_RAD(degrees) ((degrees) * 0.0174532925)
#endif

/* A hardware specific data type containing state information about on board sensors */
typedef struct {
	bool *asm330_acc_good;
	bool *asm330_gyro_good;
	bool *bmx055_acc_good;
	bool *bmx055_gyro_good;
	bool *bmx055_mag_good;
	bool *ms5611_good;
	bool *gps_good;
	bool *flash_good;
	bool *lora_good;
} Sensor_State;

typedef struct {
	flightState flight_state;
	ematchState drogue_ematch_state;
	ematchState main_ematch_state;
	armState drogue_arm_state;
	armState main_arm_state;

	uint32_t launch_time;
	float starting_altitude;
	uint32_t drogue_deploy_time;
	float drogue_deploy_altitude;
	uint32_t main_deploy_time;
	float main_deploy_altitude;
	uint32_t landing_time;
	float landing_altitude;

	float battery_voltage;
	Sensor_State *sensor_state;

	bool transmit_gps;
} System_State_FC_t;

uint8_t calculate_attitude_error(arm_matrix_instance_f32 *current_vec, arm_matrix_instance_f32 *desired_vec, float *theta, arm_matrix_instance_f32 *normal_vector);
uint8_t EP2C(arm_matrix_instance_f32 *qu, arm_matrix_instance_f32 *dir_cos);
uint8_t vector_cross_product(arm_matrix_instance_f32 *a, arm_matrix_instance_f32 *b, arm_matrix_instance_f32 *res);
bool detect_launch_accel(float *current_acc, float *prev_acc, float current_altitude);
bool detect_launch_baro();
bool detect_burnout(float *current_acc, float *prev_acc);
bool detect_apogee(float *current_vertical_velocity, float *previous_vertical_velocity);
bool detect_apogee_delay(uint32_t launch_time_us, uint32_t current_time_us);
void fill_median_filter_buffer(float data_point, size_t idx);
bool detect_landing(float *current_vertical_velocity, float *previous_vertical_velocity);
void deploy_drogue_parachute(GPIO_TypeDef *H_port, GPIO_TypeDef *L_port, uint16_t H_pin, uint16_t L_pin);
void deploy_main_parachute(GPIO_TypeDef *H_port, GPIO_TypeDef *L_port, uint16_t H_pin, uint16_t L_pin);
ematchState test_continuity(ADC_HandleTypeDef *hadc, GPIO_TypeDef *L_port, uint16_t L_pin, uint32_t adcChannel);
int compare(const void *a, const void *b);
float calculateMedian(float arr[], size_t n);
float calculateBatteryVoltage(ADC_HandleTypeDef *hadc);

#endif /* INC_STATE_MACHINE_H_ */
