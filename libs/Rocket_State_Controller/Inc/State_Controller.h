/*
 File name: State_Controller.h
 Author: Angus McLennan
 Created on: Feb 27, 2024
 Description: A state machine for use in high powered rockets to detect stages of flight

 Copyright © 2024 Angus McLennan

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef INC_STATE_CONTROLLER_H_
#define INC_STATE_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "digital_filter.h"

/*  Filter Parameters
 The parameters defined below define the operation of the state machine. If these parameters are changed, the state machine may not operate as expected.
 DO NOT CHANGE THESE PARAMETERS
 ********** FILTER PARAMETERS START **********
 */
/* Launch detection parameters */
#define LAUNCH_ACCEL_THRESHOLD 2.0f		 // g
#define LAUNCH_ACCEL_FILTER_FREQ 100.0f	 // Hz
#define LAUNCH_ACCEL_FILTER_WIDTH 20	 // Filter width elements
#define PITCH_OVER_ANGLE_THRESHOLD 30.0f // degrees

/* Burnout detection constants */
#define BURNOUT_ACCEL_FILTER_FREQ 100.0f // Hz
#define BURNOUT_ACCEL_FILTER_WIDTH 10
#define MAX_MOTOR_BURN_TIME 10.0f	 // Seconds
#define BURNOUT_ACCEL_THRESHOLD 0.0f // g

/* Apogee detection constants */
#define VERTICAL_VELOCITY_DETECT_FREQ 40 // Hz - Frequency that apogee detect algorithm runs at. Also defines the update freq of the altitude and acceleration lp filters
#define VERTICAL_VELOCITY_FILTER_FREQ 10 // Hz - Frequency that the vertical velocity median filter is updated
#define VERTICAL_VELOCITY_FILTER_WIDTH 10
#define APOGEE_DETECT_VELOCITY_THRESHOLD 1.0f // m/s
#define ALTITUDE_LP_FILTER_CUTOFF_FREQ 1.0f	  // Hz
#define ACCEL_LP_FILTER_CUTOFF_FREQ 1.0f	  // Hz
#define APOGEE_DETECT_ACCEL_THRESHOLD 2.0	  // g

// Main deploy altitude detection constants
#define MAIN_DEPLOY_ALTITUDE 300 // m above the starting altitude (ground)

// Landing detection constants
#define LANDING_SPEED_THRESHOLD 1.0f		 // m/s (magnitude)
#define LANDING_VELOCITY_LP_CUTOFF_FREQ 0.1f // Hz - Cut-off frequency of vertical velocity low pass filter used for detecting landing
#define FLIGHT_TIME_TIMOUT 3600				 // seconds

/*********** FILTER PARAMETERS END ***********/

#ifndef DEG_TO_RAD
#define DEG_TO_RAD(degrees) ((degrees) * 0.0174532925)
#endif

// Enum to define all the stages of flight
typedef enum
{
	IDLE_ON_PAD,
	LAUNCHED,
	BURNOUT,
	APOGEE,
	MAIN_CHUTE_ALTITUDE,
	LANDED,
} flightState;

// Enum to define the axis of the accelerometer that corresponds to 'out the nose' of the rocket
typedef enum
{
	X_AXIS_POSITIVE,
	X_AXIS_NEGATIVE,
	Y_AXIS_POSITIVE,
	Y_AXIS_NEGATIVE,
	Z_AXIS_POSITIVE,
	Z_AXIS_NEGATIVE
} rocketUpAxis;

// Enum to define the continuity state of an e-match
typedef enum
{
	OPEN_CIRCUIT,
	SHORT_CIRCUIT,
	GOOD,
	EMATCH_ERROR
} ematchState;

// Enum to define the arm state of an e-match
typedef enum
{
	DISARMED,
	ARMED
} armState;

typedef struct
{
	flightState flight_state;
	rocketUpAxis up_axis;
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
} System_State_t;

// A struct used to store internal state information of the state machine
// Used for debugging and logging of data
typedef struct
{
	float angle_from_vertical;
	float filtered_launch_detect_accel;
	float filtered_burnout_detect_x_axis_accel;
	float filtered_apogee_detect_altitude;
	float filtered_apogee_detect_vertical_velocity;
	float filtered_apogee_detect_accel;
	float unfiltered_main_detect_agl_altitude;
	float filtered_landing_detect_vertical_velocity;
} State_Machine_Internal_State_t;

/* Object definitions */
extern System_State_t system_state;
extern MedianFilter_t launch_median_filter;
extern MedianFilter_t burnout_median_filter;
extern MedianFilter_t apogee_median_filter;
extern ExpLowPassFilter_t altitude_exp_lp_filter;
extern ExpLowPassFilter_t accel_low_pass_filter;
extern State_Machine_Internal_State_t internal_state_fc;
extern ExpLowPassFilter_t landing_lp_filter;

/*
 Initialises the state controller
 Inputs:
 float stating_altitude: Initial barometric altitude in m
 Returns:
 1 -> Error
 0 -> Success
 */
uint8_t init_state_controller(float starting_altitude);

/*
 Detects launch using an accelerometer by comparing the magnitude of the acceleration vector to a predefined threshold
 To disable the rocket_angle check, pass in a value of 0 for rocket_angle
 Inputs:
 float ax: X-axis acceleration in g
 float ay: Y-axis acceleration in g
 float az: Y-axis acceleration in g
 float rocket_angle: Angle of rocket from vertical in radians
 uint32_t timestamp_ms: System time in miliseconds
 Returns:
 true -> Launch detected
 false -> Launch not detected
 */
bool detect_launch_accel(float ax, float ay, float az, float rocket_angle, uint32_t timestamp_ms);

/*
 Determines the up axis by reading the axis of maximum acceleration caused by the motor thrust
 This function should be fed new data in a loop until it returns true.
 float ax: X-axis acceleration in g
 float ay: Y-axis acceleration in g
 float az: Y-axis acceleration in g
 Returns:
 true -> Up axis successfully calculated
 false -> Up axis not yet calculated
 */
bool calculate_up_axis(float ax, float ay, float az);

/*
 Determines if the rocket has reached engine burnout
 This function should be fed new data in a loop
 float ax: X-axis acceleration in g
 float ay: Y-axis acceleration in g
 float az: Y-axis acceleration in g
 float altitude: Rocket's current altitude in m
 uint32_t timestamp_ms: System time in miliseconds
 Returns:
 true -> Burnout has been detected
 false -> Burnout has not been detected
 */
bool detect_burnout_accel(float ax, float ay, float az, float altitude, uint32_t timestamp_ms);

/*
 Determines if rocket has reached apogee
 This function should be fed new data in a loop
 float ax: X-axis acceleration in g
 float ay: Y-axis acceleration in g
 float az: Y-axis acceleration in g
 float altitude: Rocket's current altitude in m
 uint32_t timestamp_ms: System time in miliseconds
 Returns:
 true -> Apogee has been detected
 false -> Apogee has not been detected
 */
bool detect_apogee(float ax, float ay, float az, float altitude, uint32_t timestamp_ms);

/*
 Determines if the main parachute deployment altitude has been reached
 This function should be fed new data in a loop
 float altitude: Rocket's current altitude in m
 uint32_t timestamp_ms: System time in miliseconds
 Returns:
 true -> Main deploy altitude has been reached
 false -> Main deploy altitude has not been reached
 */
bool detect_main_deploy_altitude(float altitude, uint32_t timestamp_ms);

/*
 Determines if the vehicle has landed
 This function should be fed new data in a loop
 float altitude: Rocket's current altitude in m
 uint32_t timestamp_ms: System time in miliseconds
 Returns:
 true -> Landing has been detected
 false -> Landing has not been detected
 */
bool detect_landing(float altitude, uint32_t timestamp_ms);

#endif /* INC_STATE_CONTROLLER_H_ */
