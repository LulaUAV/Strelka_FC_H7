/*
 * State_Machine.c
 *
 *  Created on: Apr 9, 2023
 *      Author: Angus McLennan
 */

#include "State_Machine.h"

extern System_State_FC_t state_machine_fc;

/*
 * Function to detect launch of the vehicle
 * Returns:
 * false -> no launch
 * true -> launch
 */
bool detect_launch_accel(float *current_acc, float *prev_acc, float current_altitude) {
	float alpha = 0.7;
	*current_acc = (*current_acc * alpha + *prev_acc * (1 - alpha));
#ifdef USE_ALT_ARM_CHECK
	if(*current_acc > LAUNCH_THRESHOLD) {
		// Acceleration condition fulfilled, check altitude condition
		return current_altitude - system_state.starting_altitude >= LAUNCH_ALT_THRESHOLD;
	}
#else
	return *current_acc > LAUNCH_THRESHOLD;
#endif
}

/*
 * Function to detect launch of the vehicle using a change in initial altitude
 * Inputs:
 * float *current_vertical_velocity: Current vertical velocity
 * float *previous_vertical_velocity: Previous vertical velocity
 * Returns:
 * false -> no launch
 * true -> launch
 */
bool detect_launch_baro(float *current_vertical_velocity, float *previous_vertical_velocity) {
	*current_vertical_velocity = *current_vertical_velocity * BARO_LAUNCH_DETECT_ALPHA + *previous_vertical_velocity * (1 - BARO_LAUNCH_DETECT_ALPHA);
	return *current_vertical_velocity < BARO_LAUNCH_VELOCITY_THRESHOLD;
}

/*
 * Function to detect motor burnout
 * Returns:
 * false -> no burnout
 * true -> burnout
 */
bool detect_burnout(float *current_acc, float *prev_acc) {
	*current_acc = (*current_acc * BURNOUT_DETECT_ALPHA + *prev_acc * (1 - BURNOUT_DETECT_ALPHA));
	return *current_acc < BURNOUT_THRESHOLD;
}

float vertical_velocity_history[APOGEE_MEDIAN_FILTER_LENGTH] = { 0 };		// A queue of previous readings used for median filtering
/*
 * Function to detect apogee which uses a low pass filer and median filter to eliminate noise and supersonic disturbances
 * Returns:
 * false -> no apogee
 * true -> apogee
 */
bool detect_apogee(float *current_vertical_velocity, float *previous_vertical_velocity) {
	// Low pass filter velocity input
	*current_vertical_velocity = *current_vertical_velocity * APOGEE_DETECT_ALPHA + *previous_vertical_velocity * (1.0 - APOGEE_DETECT_ALPHA);

	// Shift all elements in previous velocities buffer to the right, pushing out the last element
	for (int i = APOGEE_MEDIAN_FILTER_LENGTH - 1; i > 0; i--) {
		vertical_velocity_history[i] = vertical_velocity_history[i - 1];
	}
	// Add reading to array of previous velocities
	vertical_velocity_history[0] = *current_vertical_velocity;
	float vel_hist_cpy[sizeof(vertical_velocity_history)];
	memcpy(vel_hist_cpy, vertical_velocity_history, sizeof(vertical_velocity_history) * sizeof(float));
	// Calculate median of vertical_velocity_history array
	float filtered_descent_rate = calculateMedian(vel_hist_cpy, APOGEE_MEDIAN_FILTER_LENGTH);

//	char println[64];
//	size_t szn = snprintf(println, sizeof(println), "%f,%f,%f\r\n", *current_vertical_velocity, filtered_descent_rate, APOGEE_DESCENT_RATE_THRESHOLD);
//	CDC_Transmit_HS(println, szn);

	return filtered_descent_rate < APOGEE_DESCENT_RATE_THRESHOLD;
}

void fill_median_filter_buffer(float data_point, size_t idx) {
	vertical_velocity_history[idx] = data_point;
}

/*
 * Function to detect apogee using a simple delay
 * Inputs:
 * uint32_t start_time_us: Launch time in microseconds
 * uint32_t current_time_us: Current time in microseconds
 * Returns:
 * false -> no apogee
 * true -> apogee
 */
bool detect_apogee_delay(uint32_t launch_time_us, uint32_t current_time_us) {
	return ((float) (current_time_us - launch_time_us)) / 1E6 > TIME_TO_APOGEE;
}

/*
 * Function to detect landing
 * Returns:
 * false -> no landing
 * true -> landing
 */
bool detect_landing(float *current_vertical_velocity, float *previous_vertical_velocity) {
	// Low pass filter input
	*current_vertical_velocity = *current_vertical_velocity * LANDING_DETECT_ALPHA + *previous_vertical_velocity * (1.0 - LANDING_DETECT_ALPHA);
}

// Function to compare two doubles for qsort
int compare(const void *a, const void *b) {
	return (*(float*) a > *(float*) b) - (*(float*) a < *(float*) b);
}

float calculateMedian(float arr[], size_t n) {
	// Sort the array in ascending order
	qsort(arr, n, sizeof(float), compare);

	// Check if the array has an odd or even number of elements
	if (n % 2 == 1) {
		// If odd, return the middle element
		return arr[n / 2];
	} else {
		// If even, return the average of the two middle elements
		int middle1 = n / 2 - 1;
		int middle2 = n / 2;
		return (arr[middle1] + arr[middle2]) / 2.0;
	}
}

/*
 * Function to deploy drogue parachute
 * Returns: void
 */
void deploy_drogue_parachute(GPIO_TypeDef *H_port, GPIO_TypeDef *L_port, uint16_t H_pin, uint16_t L_pin) {
	HAL_GPIO_WritePin(H_port, H_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(L_port, L_pin, GPIO_PIN_SET);
	osDelay(2000);
	HAL_GPIO_WritePin(H_port, H_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(L_port, L_pin, GPIO_PIN_RESET);
}

/*
 * Function to deploy main parachute
 * Returns: void
 */
void deploy_main_parachute(GPIO_TypeDef *H_port, GPIO_TypeDef *L_port, uint16_t H_pin, uint16_t L_pin) {
	HAL_GPIO_WritePin(H_port, H_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(L_port, L_pin, GPIO_PIN_SET);
	osDelay(2000);
	HAL_GPIO_WritePin(H_port, H_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(L_port, L_pin, GPIO_PIN_RESET);
}

ematchState test_continuity(ADC_HandleTypeDef *hadc, GPIO_TypeDef *L_port, uint16_t L_pin, uint32_t adcChannel) {
	ematchState state;
	HAL_StatusTypeDef res;

	// Set FIRE_L pin to allow for continuity sensing
	HAL_GPIO_WritePin(L_port, L_pin, GPIO_PIN_SET);

	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	if (adcChannel == ADC_CHANNEL_8) {
		sConfig.Rank = ADC_REGULAR_RANK_2;
	} else {
		sConfig.Rank = ADC_REGULAR_RANK_1;
	}
	sConfig.Channel = adcChannel;
	sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	sConfig.OffsetSignedSaturation = DISABLE;
	if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	// Start ADC
	res = HAL_ADC_Start(hadc);
	res = HAL_ADC_PollForConversion(hadc, 1);
	if (res == HAL_TIMEOUT)
		return EMATCH_ERROR;

	uint32_t AD_RES = HAL_ADC_GetValue(hadc);
	if (AD_RES > 40000) {
		state = OPEN_CIRCUIT;
	} else {
		state = GOOD;
	}

	// Stop ADC
	HAL_ADC_Stop(hadc);
	HAL_GPIO_WritePin(L_port, L_pin, GPIO_PIN_RESET);

	return state;
}

#define ADC_RESOLUTION   16
#define ADC_MAX_VALUE    ((1 << ADC_RESOLUTION) - 1)
#define V_REF            3.3  // Replace with your ADC reference voltage

// Battery voltage regression values
#define NUM_POINTS 11
#define NUM_COLS 2
float battery_adc_voltage[NUM_POINTS][NUM_COLS] = {
	{46042, 4.2},
	{45572, 4.0},
	{44922, 3.8},
	{44231, 3.6},
	{43428, 3.4},
	{42580, 3.22},
	{41691, 3.0},
	{40633, 2.8},
	{39312, 2.6},
	{38028, 2.4}
};

float interpolateBatteryVoltage(float adc_value, float battery_adc_voltage[NUM_POINTS][NUM_COLS]) {
	int idx=0;
    // Find the two closest points for linear interpolation
    for(idx=0; idx<NUM_POINTS-1; idx++) {
    	if(battery_adc_voltage[idx][0] >= adc_value && battery_adc_voltage[idx+1][0]<= adc_value) {
    		break;
    	}
    }

    // Perform linear interpolation
    float lower_adc = battery_adc_voltage[idx+1][0];
    float upper_adc = battery_adc_voltage[idx][0];
    float lower_voltage = battery_adc_voltage[idx+1][1];
    float upper_voltage = battery_adc_voltage[idx][1];

    float interpolated_voltage = lower_voltage +
        ((upper_voltage - lower_voltage) / (upper_adc - lower_adc)) * (adc_value - lower_adc);

    return interpolated_voltage;
}

float convertToVoltage(uint16_t adcValue) {
	return (adcValue / (float) ADC_MAX_VALUE) * V_REF;
}

float calculateBatteryVoltage(ADC_HandleTypeDef *hadc) {
	HAL_StatusTypeDef res;

	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
	if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	// Start ADC
	res = HAL_ADC_Start(hadc);
	res = HAL_ADC_PollForConversion(hadc, 1);
	if (res == HAL_TIMEOUT)
		return EMATCH_ERROR;

	uint16_t AD_RES = HAL_ADC_GetValue(hadc);

	// Stop ADC
	HAL_ADC_Stop(hadc);

	// Convert 16 bit value to voltage and scale by voltage divider ratio
	return interpolateBatteryVoltage(AD_RES, battery_adc_voltage);
}
