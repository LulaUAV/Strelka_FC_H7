/*
 * buzzer.h
 *
 *  Created on: Dec 16, 2023
 *      Author: thean
 */

#ifndef INC_INDICATOR_H_
#define INC_INDICATOR_H_

#include <stdbool.h>
#include "stm32h7xx_hal.h"

typedef struct {
	GPIO_TypeDef* buzzer_port;
	uint16_t buzzer_pin;
	GPIO_TypeDef* led_port;
	uint16_t led_pin;
} indicator_t;

void report_ematch_state_b(indicator_t* indicator, bool drogue_ematch, bool main_ematch);


#endif /* INC_INDICATOR_H_ */
