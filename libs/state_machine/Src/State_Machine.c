/*
 * State_Machine.c
 *
 *  Created on: Apr 9, 2023
 *      Author: Angus McLennan
 */

#include "State_Machine.h"

/*
 * Function that calculates the angular difference between the vehicle vector and the desired vector
 * Inputs:
 * current_vec: quaternion 4x1 vector representing current orientiation of vehicle
 * desired_vec: NORMALISED 3x1 vector (not quaternion) representing desired orientiation of vehicle
 * theta (output): angle value error in radians from vehicle's x-axis (pointing out the nose)
 * normal_vector (output): the normal vector of current_vec and desired_vec
 * Returns: controller_result
 */
uint8_t calculate_attitude_error(arm_matrix_instance_f32 *current_vec, arm_matrix_instance_f32 *desired_vec, float *theta, arm_matrix_instance_f32 *normal_vector)
{
	// Normalise current state vector
	float32_t qu_norm = sqrt(current_vec->pData[0] * current_vec->pData[0] + current_vec->pData[1] * current_vec->pData[1] + current_vec->pData[2] * current_vec->pData[2] + current_vec->pData[3] * current_vec->pData[3]);
	arm_matrix_instance_f32 current_vec_norm;
	float32_t current_vec_norm_data[4] = {current_vec->pData[0] / qu_norm, current_vec->pData[1] / qu_norm, current_vec->pData[2] / qu_norm, current_vec->pData[3] / qu_norm};
	arm_mat_init_f32(&current_vec_norm, 4, 1, current_vec_norm_data);

	arm_matrix_instance_f32 body_vec;
	float32_t body_vec_data[3] = {1, 0, 0};
	arm_mat_init_f32(&body_vec, 3, 1, body_vec_data);

	arm_matrix_instance_f32 rot_mat;
	float32_t rot_mat_data[9];
	arm_mat_init_f32(&rot_mat, 3, 3, rot_mat_data);

	arm_matrix_instance_f32 vec_in_body;
	float32_t vec_in_body_data[3];
	arm_mat_init_f32(&vec_in_body, 3, 1, vec_in_body_data);

	// Calculate rotation matrix from current quaternion state
	uint8_t res = EP2C(&current_vec_norm, &rot_mat);
	if (res != 0)
		return res;

	// Rotate desired_vec into frame of quaternion input. Return desired vector in body coordinates
	arm_status arm_res = arm_mat_mult_f32(&rot_mat, desired_vec, &vec_in_body);
	if (arm_res != ARM_MATH_SUCCESS)
		return arm_res;

	// Calculate error angle between current_vec and desired_vec
	float32_t dot_prod_res;
	arm_dot_prod_f32(vec_in_body.pData, body_vec.pData, 3, &dot_prod_res);
	if (dot_prod_res >= 1.0)
		dot_prod_res = 1.0;
	else if (dot_prod_res <= -1.0)
		dot_prod_res = -1.0;
	*theta = acos(dot_prod_res);

	// Calculate normal vector
	res = vector_cross_product(&vec_in_body, &body_vec, normal_vector);
	if (res != 0)
		return res;

	// Normalise normal vector
	float32_t normal_vector_magnitude = sqrt(normal_vector->pData[0] * normal_vector->pData[0] + normal_vector->pData[1] * normal_vector->pData[1] + normal_vector->pData[2] * normal_vector->pData[2]);
	normal_vector->pData[0] /= normal_vector_magnitude;
	normal_vector->pData[1] /= normal_vector_magnitude;
	normal_vector->pData[2] /= normal_vector_magnitude;

	return 0;
}

/*
 * Function takes a quaternion vector and returns the direction cosine
 * matrix in terms of the 4x1 Euler parameter vector
 * Inputs: qu - 4x1 quaternion column vector, dir_cos - 3x3 direction cosine matrix
 * Returns: controller_result
 */
uint8_t EP2C(arm_matrix_instance_f32 *qu, arm_matrix_instance_f32 *dir_cos)
{
	// Check inputs
	if (qu->numCols != 1 || qu->numRows != 4 || dir_cos->numCols != 3 || dir_cos->numRows != 3)
	{
		return 2;
	}
	float32_t q0 = qu->pData[0];
	float32_t q1 = qu->pData[1];
	float32_t q2 = qu->pData[2];
	float32_t q3 = qu->pData[3];

	dir_cos->pData[0] = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
	dir_cos->pData[1] = 2 * (q1 * q2 + q0 * q3);
	dir_cos->pData[2] = 2 * (q1 * q3 - q0 * q2);
	dir_cos->pData[3] = 2 * (q1 * q2 - q0 * q3);
	dir_cos->pData[4] = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
	dir_cos->pData[5] = 2 * (q2 * q3 + q0 * q1);
	dir_cos->pData[6] = 2 * (q1 * q3 + q0 * q2);
	dir_cos->pData[7] = 2 * (q2 * q3 - q0 * q1);
	dir_cos->pData[8] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

	return 0;
}

/*
 * Function takes two arm_matrix_instance_f32 vector pointers of size 3x1 and calculates their cross product
 * Inputs: a - 3x1 column vector, b - 3x1 column vector, res - 3x1 cross product vector
 * Returns: controller_result
 */
uint8_t vector_cross_product(arm_matrix_instance_f32 *a, arm_matrix_instance_f32 *b, arm_matrix_instance_f32 *res)
{
	if (a->numRows < 3 || b->numRows < 3 || a->numCols > 1 || b->numCols > 1)
	{
		return 2; // input vectors must have at least 3 elements
	}

	// extract values from input vectors
	float32_t x1 = a->pData[0];
	float32_t y1 = a->pData[1];
	float32_t z1 = a->pData[2];

	float32_t x2 = b->pData[0];
	float32_t y2 = b->pData[1];
	float32_t z2 = b->pData[2];

	// calculate cross product
	res->pData[0] = y1 * z2 - z1 * y2;
	res->pData[1] = z1 * x2 - x1 * z2;
	res->pData[2] = x1 * y2 - y1 * x2;
	return 0;
}

/*
 * Function to deploy drogue parachute
 * Returns: void
 */
void deploy_drogue_parachute(GPIO_TypeDef *H_port, GPIO_TypeDef *L_port, uint16_t H_pin, uint16_t L_pin)
{
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
void deploy_main_parachute(GPIO_TypeDef *H_port, GPIO_TypeDef *L_port, uint16_t H_pin, uint16_t L_pin)
{
	HAL_GPIO_WritePin(H_port, H_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(L_port, L_pin, GPIO_PIN_SET);
	osDelay(2000);
	HAL_GPIO_WritePin(H_port, H_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(L_port, L_pin, GPIO_PIN_RESET);
}

ematchState test_continuity(ADC_HandleTypeDef *hadc, GPIO_TypeDef *L_port, uint16_t L_pin, uint32_t adcChannel)
{
	ematchState state;
	HAL_StatusTypeDef res;

	// Set FIRE_L pin to allow for continuity sensing
	HAL_GPIO_WritePin(L_port, L_pin, GPIO_PIN_SET);

	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	if (adcChannel == ADC_CHANNEL_8)
	{
		sConfig.Rank = ADC_REGULAR_RANK_2;
	}
	else
	{
		sConfig.Rank = ADC_REGULAR_RANK_1;
	}
	sConfig.Channel = adcChannel;
	sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	sConfig.OffsetSignedSaturation = DISABLE;
	if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	// Start ADC
	res = HAL_ADC_Start(hadc);
	res = HAL_ADC_PollForConversion(hadc, 1);
	if (res == HAL_TIMEOUT)
		return EMATCH_ERROR;

	uint32_t AD_RES = HAL_ADC_GetValue(hadc);
	if (AD_RES > 40000)
	{
		state = OPEN_CIRCUIT;
	}
	else
	{
		state = GOOD;
	}

	// Stop ADC
	HAL_ADC_Stop(hadc);
	HAL_GPIO_WritePin(L_port, L_pin, GPIO_PIN_RESET);

	return state;
}

#define ADC_RESOLUTION 16
#define ADC_MAX_VALUE ((1 << ADC_RESOLUTION) - 1)
#define V_REF 3.3 // Replace with your ADC reference voltage

// Battery voltage regression values
#define NUM_POINTS 11
#define NUM_COLS 2
float battery_adc_voltage[NUM_POINTS][NUM_COLS] = {{46042, 4.2}, {45572, 4.0}, {44922, 3.8}, {44231, 3.6}, {43428, 3.4}, {42580, 3.22}, {41691, 3.0}, {40633, 2.8}, {39312, 2.6}, {38028, 2.4}};

float interpolateBatteryVoltage(float adc_value, float battery_adc_voltage[NUM_POINTS][NUM_COLS])
{
	int idx = 0;
	// Find the two closest points for linear interpolation
	for (idx = 0; idx < NUM_POINTS - 1; idx++)
	{
		if (battery_adc_voltage[idx][0] >= adc_value && battery_adc_voltage[idx + 1][0] <= adc_value)
		{
			break;
		}
	}

	// Perform linear interpolation
	float lower_adc = battery_adc_voltage[idx + 1][0];
	float upper_adc = battery_adc_voltage[idx][0];
	float lower_voltage = battery_adc_voltage[idx + 1][1];
	float upper_voltage = battery_adc_voltage[idx][1];

	float interpolated_voltage = lower_voltage + ((upper_voltage - lower_voltage) / (upper_adc - lower_adc)) * (adc_value - lower_adc);

	return interpolated_voltage;
}

float convertToVoltage(uint16_t adcValue)
{
	return (adcValue / (float)ADC_MAX_VALUE) * V_REF;
}

float calculateBatteryVoltage(ADC_HandleTypeDef *hadc)
{
	HAL_StatusTypeDef res;

	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
	if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
	{
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
