/*
 * Sensors.c
 *
 *  Created on: Nov 26, 2022
 *      Author: Angus McLennan
 */

#include "Sensors.h"

void init_adc(ADC_HandleTypeDef *hadc) {
	// Calibrate adc
}

float get_battery_voltage(ADC_HandleTypeDef *hadc) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;
	if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
			{
		Error_Handler();
	}
	HAL_StatusTypeDef res = HAL_ADC_Start(hadc);
	const TickType_t delay_ticks = pdMS_TO_TICKS(10);
	osDelay(delay_ticks);
	uint32_t adc_val = HAL_ADC_GetValue(hadc);
	HAL_ADC_Stop(&hadc);
	const float R1 = 13000;
	const float R2 = 47000;
	const float resistor_divider_scaling_factor = (R1 + R2) / R2;

	return (float)adc_val / ((1 << ADC_RESOLUTION_BITS) - 1) * ADC_VREF * resistor_divider_scaling_factor;
}

