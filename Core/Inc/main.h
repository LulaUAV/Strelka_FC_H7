/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SD_Det_Pin GPIO_PIN_0
#define SD_Det_GPIO_Port GPIOC
#define INDICATOR_Pin GPIO_PIN_3
#define INDICATOR_GPIO_Port GPIOC
#define GPS_PPS_Pin GPIO_PIN_0
#define GPS_PPS_GPIO_Port GPIOA
#define GPS_RST_Pin GPIO_PIN_1
#define GPS_RST_GPIO_Port GPIOA
#define BATT_SENSE_Pin GPIO_PIN_4
#define BATT_SENSE_GPIO_Port GPIOC
#define DROGUE_CONT_Pin GPIO_PIN_5
#define DROGUE_CONT_GPIO_Port GPIOC
#define MAIN_CONT_Pin GPIO_PIN_0
#define MAIN_CONT_GPIO_Port GPIOB
#define RESET_RF_Pin GPIO_PIN_1
#define RESET_RF_GPIO_Port GPIOB
#define IO0_RF_Pin GPIO_PIN_12
#define IO0_RF_GPIO_Port GPIOB
#define SPI2_NSS1_Pin GPIO_PIN_8
#define SPI2_NSS1_GPIO_Port GPIOD
#define SPI2_NSS2_Pin GPIO_PIN_9
#define SPI2_NSS2_GPIO_Port GPIOD
#define SPI2_NSS3_Pin GPIO_PIN_10
#define SPI2_NSS3_GPIO_Port GPIOD
#define SPI2_NSS4_Pin GPIO_PIN_11
#define SPI2_NSS4_GPIO_Port GPIOD
#define SPI2_NSS5_Pin GPIO_PIN_12
#define SPI2_NSS5_GPIO_Port GPIOD
#define INT_1_ASM_Pin GPIO_PIN_13
#define INT_1_ASM_GPIO_Port GPIOD
#define INT_2_ASM_Pin GPIO_PIN_14
#define INT_2_ASM_GPIO_Port GPIOD
#define INT_1_ACCEL_Pin GPIO_PIN_15
#define INT_1_ACCEL_GPIO_Port GPIOD
#define INT_1_GYRO_Pin GPIO_PIN_6
#define INT_1_GYRO_GPIO_Port GPIOC
#define DATA_READY_MAG_Pin GPIO_PIN_7
#define DATA_READY_MAG_GPIO_Port GPIOC
#define IO1_RF_Pin GPIO_PIN_3
#define IO1_RF_GPIO_Port GPIOD
#define IO2_RF_Pin GPIO_PIN_4
#define IO2_RF_GPIO_Port GPIOD
#define DROGUE_H_Pin GPIO_PIN_4
#define DROGUE_H_GPIO_Port GPIOB
#define DROGUE_L_Pin GPIO_PIN_5
#define DROGUE_L_GPIO_Port GPIOB
#define MAIN_H_Pin GPIO_PIN_6
#define MAIN_H_GPIO_Port GPIOB
#define MAIN_L_Pin GPIO_PIN_7
#define MAIN_L_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
