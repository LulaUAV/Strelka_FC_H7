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
#define LAUNCH_ACCEL_FILTER_WIDTH			10				// Filter width elements
#define LAUNCH_ACCEL_THRESHOLD 				2.0f			// g
#define PITCH_OVER_ANGLE_THRESHOLD			30.0f			// degrees

// Burnout detection constants
#define BURNOUT_ACCEL_FILTER_FREQ			100.0f			// Hz
#define MAX_MOTOR_BURN_TIME					10.0f			// Seconds
#define BURNOUT_ACCEL_THRESHOLD				-0.5f			// g

// Apogee detection constants
#define VERTICAL_VELOCITY_DETECT_FREQ		20				// Hz
#define VERTICAL_VELOCITY_FILTER_FREQ   	10				// Hz
#define APOGEE_DETECT_VELOCITY_THRESHOLD	1.0f			// m/s
#define ALTITUDE_LP_FILTER_CUTOFF_FREQ		10.0f			// Hz
#define ALTITUDE_LP_FILTER_UPDATE_FREQ		100				// Hz
#define ACCEL_LP_FILTER_UPDATE_FREQ			100				// Hz
#define ACCEL_LP_FILTER_CUTOFF_FREQ			10.0f			// Hz
#define APOGEE_DETECT_ACCEL_THRESHOLD		2.0				// g

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
	uint32_t burnout_time;
	float burnout_altitude;
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

// A struct used to store internal state information of the state machine
// Used for debugging and logging of data
typedef struct  {
	float angle_from_vertical;
	float filtered_launch_detect_accel;
	float filtered_burnout_detect_x_axis_accel;
	float filtered_apogee_detect_altitude;
	float filtered_apogee_detect_vertical_velocity;
	float filtered_apogee_detect_accel;
	float unfiltered_main_detect_agl_altitude;
	float filtered_landing_detect_vertical_velocity;
} State_Machine_Internal_State_t;

uint8_t calculate_attitude_error(arm_matrix_instance_f32 *current_vec, arm_matrix_instance_f32 *desired_vec, float *theta, arm_matrix_instance_f32 *normal_vector);
uint8_t EP2C(arm_matrix_instance_f32 *qu, arm_matrix_instance_f32 *dir_cos);
uint8_t vector_cross_product(arm_matrix_instance_f32 *a, arm_matrix_instance_f32 *b, arm_matrix_instance_f32 *res);
void deploy_drogue_parachute(GPIO_TypeDef *H_port, GPIO_TypeDef *L_port, uint16_t H_pin, uint16_t L_pin);
void deploy_main_parachute(GPIO_TypeDef *H_port, GPIO_TypeDef *L_port, uint16_t H_pin, uint16_t L_pin);
ematchState test_continuity(ADC_HandleTypeDef *hadc, GPIO_TypeDef *L_port, uint16_t L_pin, uint32_t adcChannel);
int compare(const void *a, const void *b);
float calculateMedian(float arr[], size_t n);
float calculateBatteryVoltage(ADC_HandleTypeDef *hadc);

#endif /* INC_STATE_MACHINE_H_ */
