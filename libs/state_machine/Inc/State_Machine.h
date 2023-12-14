/*
 * State_Machine.h
 *
 *  Created on: Apr 9, 2023
 *      Author: Angus McLennan
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdlib.h>

enum flightState {
	IDLE_ON_PAD, LAUNCHED, BURNOUT, APOGEE, MAIN_CHUTE_ALTITUDE, LANDED,
};

enum ematchState {
	OEPN_CIRCUIT, SHORT_CIRCUIT, GOOD,
};

/*
 * If defined, flight computer will use altitude threshold and gravity threshold
 * to detect launch. If using HPR where LAUNCH_ALT_THRESHOLD will be reached quickly,
 * enable this feature. If using LPR with TVC system where launch detection must be
 * extremely precise, disable this feature and only use acceleration to detect launch.
 ***Beware that this method is very sensitive to vibration and shock.
 */
//#define USE_ALT_ARM_CHECK

/*
 * If MOTOR_BURN_TIME is defined, the flight computer will detect motor burnout after
 * the burn time has elapsed. If this is not defined, it will detect the vertical axis
 * of the accelerometer and use the BURNOUT_THRESHOLD threshold. If the vertical accelerometer
 * detects an acceleration of less than BURNOUT_THRESHOLD, burnout is detected.
 */
#define MOTOR_BURN_TIME 				1.6 			// s
/*
 * if TIME_TO_APOGEE is defined, the flight computer will detect apogee after the time delay
 * has elapsed. If this is not defined, apogee will be detected by sensing when the vertical
 * velocity of the rocket passes below a defined threshold APOGEE_DESCENT_RATE_THRESHOLD
 */
#define TIME_TO_APOGEE					2.4f

#ifndef GRAVITY_MPS
#define GRAVITY_MPS						9.81
#endif
#define LAUNCH_THRESHOLD 				1.2				// g
#define LAUNCH_ALT_THRESHOLD 			20				// m from pad
#define BURNOUT_THRESHOLD 				-0.5*GRAVITY_MPS// g
#define APOGEE_DETECT_UPDATE_RATE		30.0f			// Hz
#define APOGEE_MEDIAN_FILTER_LENGTH		30				// Always an integer of 0.5*APOGEE_DETECT_UPDATE_RATE
#define APOGEE_DETECT_CUTOFF_FREQ		3.0f			// Hz
#define APOGEE_DETECT_ALPHA 			(1.0f-exp(-APOGEE_DETECT_CUTOFF_FREQ/APOGEE_DETECT_UPDATE_RATE))
#define BURNOUT_DETECT_CUTOFF_FREQ		0.1f
#define BURNOUT_DETECT_ALPHA			(1.0f-exp(-BURNOUT_DETECT_CUTOFF_FREQ/APOGEE_DETECT_UPDATE_RATE))
#define APOGEE_DESCENT_RATE_THRESHOLD	-0.2			// m/s
#define BARO_LAUNCH_DETECT_ALPHA		0.3
#define BARO_LAUNCH_VELOCITY_THRESHOLD	10				// m/s
#define LANDING_DETECT_ALPHA 			0.001
#define MAIN_DEPLOY_ALT 				300				// m
#define MIN_DEPLOY_ALT 					0				// m

typedef struct {
	bool transmit_gps;
	enum flightState flight_state;
	float starting_altitude;
	uint32_t launch_time;
	bool controller_saturated;
	float drogue_deploy_altitude;
	uint32_t drogue_deploy_time;
	float main_deploy_altitude;
	uint32_t main_deploy_time;
	uint32_t landing_time;
	float battery_voltage;
	enum drogue_ematch_state;
	enum main_ematch_state;
	bool fire_drogue_ready;
	bool fire_main_ready;
	bool fire_valve_ready;
	bool flash_good;
} System_State;

bool detect_launch_accel(float* current_acc, float* prev_acc, float current_altitude);
bool detect_launch_baro();
bool detect_burnout(float* current_acc, float* prev_acc);
bool detect_apogee(float* current_vertical_velocity, float* previous_vertical_velocity);
bool detect_apogee_delay(uint32_t launch_time_us, uint32_t current_time_us);
void fill_median_filter_buffer(float data_point, size_t idx);
bool detect_landing(float* current_vertical_velocity, float* previous_vertical_velocity);
void deploy_drogue_parachute(GPIO_TypeDef* H_port, GPIO_TypeDef* L_port, uint16_t H_pin, uint16_t L_pin);
void deploy_main_parachute(GPIO_TypeDef* H_port, GPIO_TypeDef* L_port, uint16_t H_pin, uint16_t L_pin);
int compare(const void *a, const void *b);
float calculateMedian(float arr[], size_t n);

#endif /* INC_STATE_MACHINE_H_ */
