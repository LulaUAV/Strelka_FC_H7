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
#include "State_Controller.h"


#ifndef GRAVITY_MPS
#define GRAVITY_MPS							9.81
#endif


#ifndef DEG_TO_RAD
#define DEG_TO_RAD(degrees) ((degrees) * 0.0174532925)
#endif

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
