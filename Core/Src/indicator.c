/*
 * buzzer.c
 *
 *  Created on: Dec 16, 2023
 *      Author: thean
 */

#include "indicator.h"

void report_ematch_state_b(indicator_t *indicator, bool drogue_ematch, bool main_ematch) {
	if (drogue_ematch & main_ematch) {
		// Pulse three times
		for (int i = 0; i < 3; i++) {
			HAL_GPIO_WritePin(indicator->buzzer_port, indicator->buzzer_pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(indicator->led_port, indicator->led_pin, GPIO_PIN_SET);
			osDelay(500);
			HAL_GPIO_WritePin(indicator->buzzer_port, indicator->buzzer_pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(indicator->led_port, indicator->led_pin, GPIO_PIN_RESET);
			osDelay(500);
		}
	} else if (drogue_ematch) {
		// Pulse two times
		for (int i = 0; i < 2; i++) {
			HAL_GPIO_WritePin(indicator->buzzer_port, indicator->buzzer_pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(indicator->led_port, indicator->led_pin, GPIO_PIN_SET);
			osDelay(500);
			HAL_GPIO_WritePin(indicator->buzzer_port, indicator->buzzer_pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(indicator->led_port, indicator->led_pin, GPIO_PIN_RESET);
			osDelay(500);
		}

	} else if (main_ematch) {
		// Pulse once
		HAL_GPIO_WritePin(indicator->buzzer_port, indicator->buzzer_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(indicator->led_port, indicator->led_pin, GPIO_PIN_SET);
		osDelay(500);
		HAL_GPIO_WritePin(indicator->buzzer_port, indicator->buzzer_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(indicator->led_port, indicator->led_pin, GPIO_PIN_RESET);
		osDelay(500);

	} else {
		// Pulse once long
		HAL_GPIO_WritePin(indicator->buzzer_port, indicator->buzzer_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(indicator->led_port, indicator->led_pin, GPIO_PIN_SET);
		osDelay(2000);
		HAL_GPIO_WritePin(indicator->buzzer_port, indicator->buzzer_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(indicator->led_port, indicator->led_pin, GPIO_PIN_RESET);
		osDelay(2000);
	}
}
