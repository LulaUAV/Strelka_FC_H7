/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMX055.h"
#include "MS5611.h"
#include "Sensors.h"
#include "LoRa.h"
#include "ASM330.h"
#include "debug.h"
#include "gps.h"
#include "State_Machine.h"
#include "Packets_Definitions.h"
#include "SD.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c2;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

MDMA_HandleTypeDef hmdma_mdma_channel0_sdmmc1_end_data_0;
MDMA_LinkNodeTypeDef node_mdma_channel0_sdmmc1_command_end_1;
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 2048 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for State_Machine_T */
osThreadId_t State_Machine_THandle;
uint32_t myTask02Buffer[ 2048 ];
osStaticThreadDef_t myTask02ControlBlock;
const osThreadAttr_t State_Machine_T_attributes = {
  .name = "State_Machine_T",
  .cb_mem = &myTask02ControlBlock,
  .cb_size = sizeof(myTask02ControlBlock),
  .stack_mem = &myTask02Buffer[0],
  .stack_size = sizeof(myTask02Buffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Sample_Sensors_ */
osThreadId_t Sample_Sensors_Handle;
uint32_t Sample_Sensors_Buffer[ 2048 ];
osStaticThreadDef_t Sample_Sensors_ControlBlock;
const osThreadAttr_t Sample_Sensors__attributes = {
  .name = "Sample_Sensors_",
  .cb_mem = &Sample_Sensors_ControlBlock,
  .cb_size = sizeof(Sample_Sensors_ControlBlock),
  .stack_mem = &Sample_Sensors_Buffer[0],
  .stack_size = sizeof(Sample_Sensors_Buffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LoRa_Task */
osThreadId_t LoRa_TaskHandle;
uint32_t LoRa_TaskBuffer[ 2048 ];
osStaticThreadDef_t LoRa_TaskControlBlock;
const osThreadAttr_t LoRa_Task_attributes = {
  .name = "LoRa_Task",
  .cb_mem = &LoRa_TaskControlBlock,
  .cb_size = sizeof(LoRa_TaskControlBlock),
  .stack_mem = &LoRa_TaskBuffer[0],
  .stack_size = sizeof(LoRa_TaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Sample_Baro_Tas */
osThreadId_t Sample_Baro_TasHandle;
uint32_t Sample_Baro_TasBuffer[ 2048 ];
osStaticThreadDef_t Sample_Baro_TasControlBlock;
const osThreadAttr_t Sample_Baro_Tas_attributes = {
  .name = "Sample_Baro_Tas",
  .cb_mem = &Sample_Baro_TasControlBlock,
  .cb_size = sizeof(Sample_Baro_TasControlBlock),
  .stack_mem = &Sample_Baro_TasBuffer[0],
  .stack_size = sizeof(Sample_Baro_TasBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Data_Logging_Ta */
osThreadId_t Data_Logging_TaHandle;
uint32_t Data_Logging_TaBuffer[ 2048 ];
osStaticThreadDef_t Data_Logging_TaControlBlock;
const osThreadAttr_t Data_Logging_Ta_attributes = {
  .name = "Data_Logging_Ta",
  .cb_mem = &Data_Logging_TaControlBlock,
  .cb_size = sizeof(Data_Logging_TaControlBlock),
  .stack_mem = &Data_Logging_TaBuffer[0],
  .stack_size = sizeof(Data_Logging_TaBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for GPS_Tracker_Tas */
osThreadId_t GPS_Tracker_TasHandle;
uint32_t GPS_Tracker_TasBuffer[ 2048 ];
osStaticThreadDef_t GPS_Tracker_TasControlBlock;
const osThreadAttr_t GPS_Tracker_Tas_attributes = {
  .name = "GPS_Tracker_Tas",
  .cb_mem = &GPS_Tracker_TasControlBlock,
  .cb_size = sizeof(GPS_Tracker_TasControlBlock),
  .stack_mem = &GPS_Tracker_TasBuffer[0],
  .stack_size = sizeof(GPS_Tracker_TasBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_MDMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_CRC_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_TIM13_Init(void);
static void MX_SPI1_Init(void);
static void MX_FDCAN1_Init(void);
void StartDefaultTask(void *argument);
void State_Machine(void *argument);
void Sample_Sensors(void *argument);
void LoRa_Radio(void *argument);
void Sample_Baro(void *argument);
void Data_Logging(void *argument);
void GPS_Tracker(void *argument);

/* USER CODE BEGIN PFP */
enum debug_level dbg_level = INFO;
enum debug_level dbg;

BMX055_Handle bmx055 = { .hspi = &hspi2, .acc_CS_port = SPI2_NSS1_GPIO_Port, .acc_CS_pin = SPI2_NSS1_Pin, .acc_range = BMX055_ACC_RANGE_8, .acc_bandwidth = BMX055_ACC_PMU_BW_62_5, .gyro_CS_port = SPI2_NSS2_GPIO_Port, .gyro_CS_pin = SPI2_NSS2_Pin, .gyro_range = BMX055_GYRO_RANGE_32_8, .gyro_bandwidth = BMX055_GYRO_BW_64, .mag_CS_port = SPI2_NSS3_GPIO_Port, .mag_CS_pin = SPI2_NSS3_Pin, .mag_data_rate = BMX055_MAG_DATA_RATE_30,

};
bool sensors_initialised;
uint32_t device_hardware_id;
BMX055_Data_Handle bmx055_data = { 0 };
MS5611_Data_Handle ms5611_data = { 0 };
ASM330_Data_Handle asm330_data = { 0 };
GPS_Data_Handle gps_data = { 0 };
GPS_Handle gps = { .gps_good = false, .gps_buffer = { 0 } };
LoRa LoRa_Handle;
MS5611_Handle ms5611 = { .hspi = &hspi4, .baro_CS_port = SPI4_NSS_GPIO_Port, .baro_CS_pin = SPI4_NSS_Pin, };
ms5611_osr_t osr = MS5611_ULTRA_HIGH_RES;
SD_Handle_t SD_card = { .flash_good = false, .log_frequency = 100 };
ASM330_handle asm330 = { .hspi = &hspi2, .CS_GPIO_Port = SPI2_NSS4_GPIO_Port, .CS_Pin = SPI2_NSS4_Pin, .accel_odr = ASM330LHHX_XL_ODR_6667Hz, .accel_scale = ASM330LHHX_8g, .gyro_odr = ASM330LHHX_GY_ODR_6667Hz, .gyro_scale = ASM330LHHX_4000dps, .acc_good = false, .gyro_good = false, };
Sensor_State sensor_state = { .asm330_acc_good = (bool*) &asm330.acc_good, .asm330_gyro_good = (bool*) &asm330.gyro_good, .bmx055_acc_good = &bmx055.acc_good, .bmx055_gyro_good = &bmx055.gyro_good, .bmx055_mag_good = &bmx055.mag_good, .flash_good = &SD_card.flash_good, .gps_good = &gps.gps_good, .lora_good = &LoRa_Handle.lora_good, .ms5611_good = &ms5611.baro_good, };
System_State_FC_t state_machine_fc = { .transmit_gps = true, .sensor_state = &sensor_state, };
GPS_Tracking_Handle gps_tracker = { .tracking_enabled = false, .chirp_frequency = 1 };

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
// Give notification to Sample_Sensors_Handle so that scheduler enables the task
//	vTaskNotifyGiveFromISR(Sample_Sensors_Handle, NULL);
	switch (GPIO_Pin) {
	case INT_1_ASM_Pin:
		xTaskNotifyFromISR(Sample_Sensors_Handle, (uint32_t )ASM330_Accel, eSetBits, NULL);
		break;
	case INT_2_ASM_Pin:
		xTaskNotifyFromISR(Sample_Sensors_Handle, (uint32_t )ASM330_Gyro, eSetBits, NULL);
		break;
	case INT_1_ACCEL_Pin:
		/* Accelerometer interrupt */
		// Log time of interrupt
		xTaskNotifyFromISR(Sample_Sensors_Handle, (uint32_t )BMX055_Accel, eSetBits, NULL);
		return;
	case INT_1_GYRO_Pin:
		/* Gyroscope interrupt */
		// Log time of interrupt
		xTaskNotifyFromISR(Sample_Sensors_Handle, (uint32_t )BMX055_Gyro, eSetBits, NULL);
		return;
	case DATA_READY_MAG_Pin:
		/* Magnetometer interrupt */
		// Log time of interrupt
		xTaskNotifyFromISR(Sample_Sensors_Handle, (uint32_t )BMX055_Mag, eSetBits, NULL);
		return;
	case GPS_PPS_Pin:
		gps.gps_good = true;
		// Reset 1.5s timer. If it elapses, GPS fix lost
		TIM2->CNT = 0;
		break;
	case IO0_RF_Pin:
		/* LoRa interrupt */
		xTaskNotifyFromISR(LoRa_TaskHandle, NULL, eNoAction, NULL);
		break;
	case IO1_RF_Pin:
		break;
	case IO2_RF_Pin:
		break;
	default:
		return;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_MDMA_Init();
  MX_SPI2_Init();
  MX_SPI4_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  MX_SDMMC1_SD_Init();
  MX_TIM13_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
	// Used to ensure all priority grouping are pre-emption (no sub-priorities) so that CMSIS does not fault
	NVIC_SetPriorityGrouping(0);

	// Read device hardware ID
	device_hardware_id = DBGMCU->IDCODE;

	/* LoRa configurations */
	LoRa_Handle = newLoRa();
	LoRa_Handle.hSPIx = &hspi2;
	LoRa_Handle.CS_port = SPI2_NSS5_GPIO_Port;
	LoRa_Handle.CS_pin = SPI2_NSS5_Pin;
	LoRa_Handle.reset_port = RESET_RF_GPIO_Port;
	LoRa_Handle.reset_pin = RESET_RF_Pin;
	LoRa_Handle.DIO0_port = IO0_RF_GPIO_Port;
	LoRa_Handle.DIO0_pin = IO0_RF_Pin;

	LoRa_Handle.frequency = 915;
	LoRa_Handle.spredingFactor = SF_7;		 // default = SF_7
	LoRa_Handle.bandWidth = BW_125KHz;		 // default = BW_125KHz
	LoRa_Handle.crcRate = CR_4_5;			 // default = CR_4_5
	LoRa_Handle.power = POWER_20db;			 // default = 20db
	LoRa_Handle.overCurrentProtection = 100; // default = 100 mA
	LoRa_Handle.preamble = 8;				 // default = 8;

	HAL_GPIO_WritePin(SPI2_NSS5_GPIO_Port, SPI2_NSS5_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of State_Machine_T */
  State_Machine_THandle = osThreadNew(State_Machine, NULL, &State_Machine_T_attributes);

  /* creation of Sample_Sensors_ */
  Sample_Sensors_Handle = osThreadNew(Sample_Sensors, NULL, &Sample_Sensors__attributes);

  /* creation of LoRa_Task */
  LoRa_TaskHandle = osThreadNew(LoRa_Radio, NULL, &LoRa_Task_attributes);

  /* creation of Sample_Baro_Tas */
  Sample_Baro_TasHandle = osThreadNew(Sample_Baro, NULL, &Sample_Baro_Tas_attributes);

  /* creation of Data_Logging_Ta */
  Data_Logging_TaHandle = osThreadNew(Data_Logging, NULL, &Data_Logging_Ta_attributes);

  /* creation of GPS_Tracker_Tas */
  GPS_Tracker_TasHandle = osThreadNew(GPS_Tracker, NULL, &GPS_Tracker_Tas_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV4;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 16;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 0;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 4;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 120-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4000000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 60000-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 3000;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * Enable MDMA controller clock
  * Configure MDMA for global transfers
  *   hmdma_mdma_channel0_sdmmc1_end_data_0
  *   node_mdma_channel0_sdmmc1_command_end_1
  */
static void MX_MDMA_Init(void)
{

  /* MDMA controller clock enable */
  __HAL_RCC_MDMA_CLK_ENABLE();
  /* Local variables */
  MDMA_LinkNodeConfTypeDef nodeConfig;

  /* Configure MDMA channel MDMA_Channel0 */
  /* Configure MDMA request hmdma_mdma_channel0_sdmmc1_end_data_0 on MDMA_Channel0 */
  hmdma_mdma_channel0_sdmmc1_end_data_0.Instance = MDMA_Channel0;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.Request = MDMA_REQUEST_SDMMC1_END_DATA;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.TransferTriggerMode = MDMA_BUFFER_TRANSFER;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.Priority = MDMA_PRIORITY_LOW;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.BufferTransferLength = 1;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel0_sdmmc1_end_data_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel0_sdmmc1_end_data_0, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initialize MDMA link node according to specified parameters */
  nodeConfig.Init.Request = MDMA_REQUEST_SDMMC1_COMMAND_END;
  nodeConfig.Init.TransferTriggerMode = MDMA_BUFFER_TRANSFER;
  nodeConfig.Init.Priority = MDMA_PRIORITY_LOW;
  nodeConfig.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  nodeConfig.Init.SourceInc = MDMA_SRC_INC_BYTE;
  nodeConfig.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  nodeConfig.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  nodeConfig.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  nodeConfig.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  nodeConfig.Init.BufferTransferLength = 1;
  nodeConfig.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  nodeConfig.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  nodeConfig.Init.SourceBlockAddressOffset = 0;
  nodeConfig.Init.DestBlockAddressOffset = 0;
  nodeConfig.PostRequestMaskAddress = 0;
  nodeConfig.PostRequestMaskData = 0;
  nodeConfig.SrcAddress = 0;
  nodeConfig.DstAddress = 0;
  nodeConfig.BlockDataLength = 0;
  nodeConfig.BlockCount = 0;
  if (HAL_MDMA_LinkedList_CreateNode(&node_mdma_channel0_sdmmc1_command_end_1, &nodeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN mdma_channel0_sdmmc1_command_end_1 */

  /* USER CODE END mdma_channel0_sdmmc1_command_end_1 */

  /* Connect a node to the linked list */
  if (HAL_MDMA_LinkedList_AddNode(&hmdma_mdma_channel0_sdmmc1_end_data_0, &node_mdma_channel0_sdmmc1_command_end_1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* MDMA interrupt initialization */
  /* MDMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MDMA_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(MDMA_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(INDICATOR_GPIO_Port, INDICATOR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RESET_RF_Pin|DROGUE_H_Pin|DROGUE_L_Pin|MAIN_H_Pin
                          |MAIN_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI4_NSS_GPIO_Port, SPI4_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_NSS1_GPIO_Port, SPI2_NSS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SPI2_NSS2_Pin|SPI2_NSS3_Pin|SPI2_NSS4_Pin|SPI2_NSS5_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_Det_Pin */
  GPIO_InitStruct.Pin = SD_Det_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SD_Det_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INDICATOR_Pin */
  GPIO_InitStruct.Pin = INDICATOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(INDICATOR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_PPS_Pin */
  GPIO_InitStruct.Pin = GPS_PPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPS_PPS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_RST_Pin */
  GPIO_InitStruct.Pin = GPS_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPS_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_RF_Pin DROGUE_H_Pin DROGUE_L_Pin MAIN_H_Pin
                           MAIN_L_Pin */
  GPIO_InitStruct.Pin = RESET_RF_Pin|DROGUE_H_Pin|DROGUE_L_Pin|MAIN_H_Pin
                          |MAIN_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI4_NSS_Pin */
  GPIO_InitStruct.Pin = SPI4_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI4_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IO0_RF_Pin */
  GPIO_InitStruct.Pin = IO0_RF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IO0_RF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_NSS1_Pin SPI2_NSS2_Pin SPI2_NSS3_Pin SPI2_NSS4_Pin
                           SPI2_NSS5_Pin */
  GPIO_InitStruct.Pin = SPI2_NSS1_Pin|SPI2_NSS2_Pin|SPI2_NSS3_Pin|SPI2_NSS4_Pin
                          |SPI2_NSS5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_1_ASM_Pin INT_2_ASM_Pin INT_1_ACCEL_Pin IO1_RF_Pin
                           IO2_RF_Pin */
  GPIO_InitStruct.Pin = INT_1_ASM_Pin|INT_2_ASM_Pin|INT_1_ACCEL_Pin|IO1_RF_Pin
                          |IO2_RF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_1_GYRO_Pin DATA_READY_MAG_Pin */
  GPIO_InitStruct.Pin = INT_1_GYRO_Pin|DATA_READY_MAG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_DMA(&huart2, gps_data.gps_buffer, sizeof(gps_data.gps_buffer));
	xTaskNotifyFromISR(Sample_Sensors_Handle, (uint32_t )MAX_10S_GPS, eSetBits, NULL);
//	debug_print(gps_buffer, sizeof(gps_buffer) , dbg=INFO);
}

/****** Radio control packet handling functions ******/
void handle_rf_rx_packet(uint8_t *Rx_buffer, size_t len) {
	uint16_t identifier;
	memcpy(&identifier, Rx_buffer, 2);
	uint8_t protocol_version;
	memcpy(&protocol_version, &Rx_buffer[2], 1);
	uint32_t receiver_hardware_id;
	uint32_t sender_hardware_id;
	memcpy(&sender_hardware_id, &Rx_buffer[3], 4);
	memcpy(&receiver_hardware_id, &Rx_buffer[7], 4);

	uint8_t payload_len = get_rf_payload_len(identifier);

	// Extract CRC bytes
	uint32_t incoming_crc32;
	memcpy(&incoming_crc32, &Rx_buffer[11 + payload_len], 4);

	// Calculate CRC on all data up to CRC32 bytes
	uint32_t calculated_crc32 = Calculate_CRC32(&hcrc, Rx_buffer, len);

	// Check that CRC32s match
	if (incoming_crc32 != calculated_crc32) {
		return;
	}

	if (protocol_version == 0) {
		// Check that sender is the ground station
		if (sender_hardware_id != 0x00000000) {
			return;
		}
		// Check that packet is intended for this device
		if (receiver_hardware_id != device_hardware_id) {
			return;
		}

		handle_payload_data(identifier, &Rx_buffer[11]);
	} else {
		// Invalid protocol specified
		return;
	}
}

uint8_t get_rf_payload_len(uint8_t identifier) {
	switch (identifier) {
	case FLASH_MEMORY_CONFIG_SET:
		// TODO: Define this packet
		return FLASH_MEMORY_CONFIG_SET_PKT_LEN;
	case GPS_TRACKING_CONFIG_SET:
		return GPS_TRACKING_CONFIG_SET_PKT_LEN;
	default:
		return 0;
	}
}

uint32_t Calculate_CRC32(CRC_HandleTypeDef *hcrc, uint8_t *payload_data, size_t len) {
	uint32_t *payload = (uint32_t*) malloc((len - 4) * sizeof(uint32_t));
	// Copy payload_data into uint32_t array without including the last 4 bytes (the CRC)
	for (int i = 0; i < len - sizeof(uint32_t); i++) {
		payload[i] = payload_data[i];
	}
	uint32_t crc = HAL_CRC_Calculate(hcrc, (uint32_t*) payload, (uint32_t) len - sizeof(uint32_t));
	free(payload);
	return crc;
}

void handle_payload_data(uint8_t identifier, uint8_t *payload_data) {
	switch (identifier) {
	case BAT_VOL_REQ:
		bat_vol_res bat_vol_pkt = { .battery_voltage = 0 /* TODO: Add data here */};
		send_rf_packet(BAT_VOL_RES, (uint8_t*) &bat_vol_pkt, sizeof(bat_vol_pkt));
		break;
	case CONTINUITY_REQ:
		continuity_res cont_pkt;
		// Read drogue continuity
		state_machine_fc.drogue_ematch_state = test_continuity(&hadc1, DROGUE_L_GPIO_Port, DROGUE_L_Pin);
		// Read main continuity
		state_machine_fc.main_ematch_state = test_continuity(&hadc1, MAIN_L_GPIO_Port, MAIN_L_Pin);
		cont_pkt.drogue_ematch_state = state_machine_fc.drogue_ematch_state;
		cont_pkt.main_ematch_state = state_machine_fc.main_ematch_state;
		send_rf_packet(CONTINUITY_REQ, (uint8_t*) &cont_pkt, sizeof(cont_pkt));
		break;
	case FIRE_DROGUE_REQ:
		fire_drogue_res fire_drogue_pkt = { .result = 0 };
		deploy_drogue_parachute(DROGUE_H_GPIO_Port, DROGUE_L_GPIO_Port, DROGUE_H_Pin, DROGUE_L_Pin);
		send_rf_packet(FIRE_DROGUE_RES, (uint8_t*) &fire_drogue_pkt, sizeof(fire_drogue_pkt));
		break;
	case FIRE_MAIN_REQ:
		fire_main_res fire_main_pkt = { .result = 0 };
		deploy_main_parachute(MAIN_H_GPIO_Port, MAIN_L_GPIO_Port, MAIN_H_Pin, MAIN_L_Pin);
		send_rf_packet(FIRE_MAIN_RES, (uint8_t*) &fire_main_pkt, sizeof(fire_main_pkt));
		break;
	case GPS1_STATE_REQ:
		gps1_state_res gps1_state_pkt = { .gps_good = gps.gps_good, .latitude = minmea_tocoord(&gps.gga_frame.latitude), .longitude = minmea_tocoord(&gps.gga_frame.longitude), .altitude = minmea_tofloat(&gps.gga_frame.altitude), };
		send_rf_packet(GPS1_STATE_RES, (uint8_t*) &gps1_state_pkt, sizeof(gps1_state_pkt));
		break;
	case GPS2_STATE_REQ:
		break;
	case ACCEL1_STATE_REQ:
		accel1_state_res accel1_state_pkt = { .acc_good = bmx055.acc_good, .accX = bmx055_data.accel[0], .accY = bmx055_data.accel[1], .accZ = bmx055_data.accel[2], };
		send_rf_packet(ACCEL1_STATE_RES, (uint8_t*) &accel1_state_pkt, sizeof(accel1_state_pkt));
		break;
	case ACCEL2_STATE_REQ:
		accel2_state_res accel2_state_pkt = { .acc_good = asm330.acc_good, .accX = asm330_data.accel[0], .accY = asm330_data.accel[1], .accZ = asm330_data.accel[2], };
		send_rf_packet(ACCEL2_STATE_RES, (uint8_t*) &accel2_state_pkt, sizeof(accel2_state_pkt));
		break;
	case GYRO1_STATE_REQ:
		gyro1_state_res gyro1_state_pkt = { .gyro_good = bmx055.gyro_good, .gyroX = bmx055_data.gyro[0], .gyroY = bmx055_data.gyro[1], .gyroZ = bmx055_data.gyro[2], };
		send_rf_packet(GYRO1_STATE_RES, (uint8_t*) &gyro1_state_pkt, sizeof(gyro1_state_pkt));
		break;
	case GYRO2_STATE_REQ:
		gyro2_state_res gyro2_state_pkt = { .gyro_good = asm330.gyro_good, .gyroX = asm330_data.gyro[0], .gyroY = asm330_data.gyro[1], .gyroZ = asm330_data.gyro[2], };
		send_rf_packet(GYRO2_STATE_RES, (uint8_t*) &gyro2_state_pkt, sizeof(gyro2_state_pkt));
		break;
	case MAG1_STATE_REQ:
		mag1_state_res mag1_state_pkt = { .mag_good = bmx055.mag_good, .magX = bmx055_data.mag[0], .magY = bmx055_data.mag[1], .magZ = bmx055_data.mag[2], };
		send_rf_packet(MAG1_STATE_RES, (uint8_t*) &mag1_state_pkt, sizeof(mag1_state_pkt));
		break;
	case MAG2_STATE_REQ:
		break;
	case BARO1_STATE_REQ:
		baro1_state_res baro1_state_pkt = { .baro_good = ms5611.baro_good, .pressure = ms5611_data.pressure, .temperature = ms5611_data.temperature, .altitude = ms5611_data.altitude, };
		send_rf_packet(BARO1_STATE_RES, (uint8_t*) &baro1_state_pkt, sizeof(baro1_state_pkt));
		break;
	case BARO2_STATE_REQ:
		break;
	case FLASH_MEMORY_STATE_REQ:
		float available_flash_memory_kB;
		FRESULT res = SD_get_free_space_kB(&available_flash_memory_kB);
		if (res != FR_OK) {
			SD_card.flash_good = false;
			// TODO: Handle error
		}
		flash_state_res flash_state_pkt = { .flash_good = SD_card.flash_good, .write_speed = SD_card.log_frequency, .available_space = available_flash_memory_kB, };
		send_rf_packet(FLASH_MEMORY_STATE_RES, (uint8_t*) &flash_state_pkt, sizeof(flash_state_pkt));
		break;
	case FLASH_MEMORY_CONFIG_SET:
		// TODO: Add input protection for bad inputs
		flash_memory_config_set flash_memory_config;
		memcpy(&flash_memory_config, payload_data, sizeof(flash_memory_config));
		SD_card.log_frequency = flash_memory_config.write_speed;
		break;
	case GPS_TRACKING_CONFIG_REQ:
		gps_tracking_config_res gps_tracking_config_pkt = { .tracking_enabled = (uint8_t) gps_tracker.tracking_enabled, .chirp_frequency = gps_tracker.chirp_frequency, .gps_good = gps.gps_good, };
		send_rf_packet(GPS_TRACKING_CONFIG_RES, (uint8_t*) &gps_tracking_config_pkt, sizeof(gps_tracking_config_pkt));
		break;
	case GPS_TRACKING_CONFIG_SET:
		// TODO: Add input protection for bad inputs
		gps_tracking_config_set gps_tracking_config;
		memcpy(&gps_tracking_config, payload_data, sizeof(gps_tracking_config));
		gps_tracker.tracking_enabled = gps_tracking_config.tracking_enabled;
		gps_tracker.chirp_frequency = gps_tracking_config.chirp_frequency;
		break;
	case STREAM_PKT_CONFIG_SET:
		stream_packet_config_set stream_pkt_config;
		memcpy(&stream_pkt_config, payload_data, sizeof(stream_pkt_config));
		// TODO: Use data in config packet to change streaming settings
		break;
	}
}

void send_rf_packet(uint16_t identifier, uint8_t *payload_data, size_t len) {
	uint8_t *send_pkt = (uint8_t*) malloc(len + 14);
	memcpy(&send_pkt[0], &identifier, sizeof(identifier));
	uint32_t sender_unique_id = device_hardware_id;
	uint32_t receiver_unique_id = 0x00000000;		// Ground station ID
	memcpy(&send_pkt[2], &sender_unique_id, 4);
	memcpy(&send_pkt[6], &receiver_unique_id, 4);
	memcpy(&send_pkt[10], payload_data, len);
	uint32_t crc32 = Calculate_CRC32(&hcrc, send_pkt, len + 14);
	memcpy(&send_pkt[len + 10], &crc32, 4);
	uint8_t res = LoRa_transmit(&LoRa_Handle, send_pkt, len + 14, 1000);
	if (res) {
		// TODO: Handle LoRa timeout
		printf("LoRa timed out");
	}
	LoRa_startReceiving(&LoRa_Handle);
}

/****** Radio control packet handling END *****/

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(10000);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_State_Machine */
/**
 * @brief Function implementing the State_Machine_T thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_State_Machine */
void State_Machine(void *argument)
{
  /* USER CODE BEGIN State_Machine */
	// Calibrate ADC for better accuracy
//	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
//
//	while (!sensors_initialised) {
//		osDelay(10);
//	}
//
//	state_machine_fc.flight_state = IDLE_ON_PAD;
//	state_machine_fc.starting_altitude = ms5611_data.altitude;
//
//	// Check drogue continuity
//	state_machine_fc.drogue_ematch_state = test_continuity(&hadc1, DROGUE_L_GPIO_Port, DROGUE_L_Pin);
//
//	// Check main continuity
//	state_machine_fc.main_ematch_state = test_continuity(&hadc1, MAIN_L_GPIO_Port, MAIN_L_Pin);
	// Report E-match state
	// TODO report ematch state over buzzer
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
  /* USER CODE END State_Machine */
}

/* USER CODE BEGIN Header_Sample_Sensors */
/**
 * @brief Function implementing the Sample_Sensors_ thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Sample_Sensors */
void Sample_Sensors(void *argument)
{
  /* USER CODE BEGIN Sample_Sensors */
	sensors_initialised = false;

	/* Init BMX055 */
	if (!BMX055_init(&bmx055)) {
//		debug_print("BMX055 FAILED\r\n", sizeof("BMX055 FAILED\r\n"), dbg =
//				CRITICAL);
		printf("Error");
	}
	// Configure interrupts
	BMX055_setInterrupts(&bmx055);

	/* Init ASM330 */
	if (ASM330_Init(&asm330)) {
		// TODO: Handle error
		printf("Error");
	}

	/* Init GPS */
	HAL_StatusTypeDef res = HAL_UART_Receive_DMA(&huart2, gps_data.gps_buffer, sizeof(gps_data.gps_buffer));

	/* Enable interrupts */
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* Perform system checks before arming */
	// Check e-match continuities
	// Check critical sensors
	if ((bmx055.acc_good == false && asm330.acc_good == false) || ms5611.baro_good == false) {
		// Alert critical sensor error code
	}

	// Sensor type that is ready when task is released
	uint32_t sensor_type;
	/* Infinite loop */
	for (;;) {
		// Wait for sensors to be ready before running task
		xTaskNotifyWait(0, 0, &sensor_type, (TickType_t) portMAX_DELAY);
		/* Check each sensor each loop for new data */
#ifndef RUN_HITL
		// Check BMX055_Accel
		if (sensor_type & BMX055_Accel) {
			// Clear bits corresponding to this case
			ulTaskNotifyValueClear(Sample_Sensors_Handle, BMX055_Accel);
			float accel_data[3];
			BMX055_readAccel(&bmx055, accel_data);
			BMX055_exp_filter(bmx055_data.accel, accel_data, bmx055_data.accel, sizeof(accel_data) / sizeof(int),
			ACCEL_ALPHA);
		}
		// Check BMX055_Gyro
		if (sensor_type & BMX055_Gyro) {
			// Clear bits corresponding to this case
			ulTaskNotifyValueClear(Sample_Sensors_Handle, BMX055_Gyro);
			float gyro_data[3];
			BMX055_readGyro(&bmx055, gyro_data);
			BMX055_exp_filter(bmx055_data.gyro, gyro_data, bmx055_data.gyro, sizeof(gyro_data) / sizeof(int),
			GYRO_ALPHA);
		}
		// Check BMX055_Mag
		if (sensor_type & BMX055_Mag) {
			// Clear bits corresponding to this case
			ulTaskNotifyValueClear(Sample_Sensors_Handle, BMX055_Mag);
			BMX055_readCompensatedMag(&bmx055, bmx055_data.mag);
		}
		// Check asm330_Accel
		if (sensor_type & ASM330_Accel) {
			// Clear bits corresponding to this case
			ulTaskNotifyValueClear(Sample_Sensors_Handle, ASM330_Accel);
			if (ASM330_readAccel(&asm330, asm330_data.accel)) {
				// TODO: Handle error
			}
		}
		// Check asm330_Gyro
		if (sensor_type & ASM330_Gyro) {
			// Clear bits corresponding to this case
			ulTaskNotifyValueClear(Sample_Sensors_Handle, ASM330_Gyro);
			if (ASM330_readGyro(&asm330, asm330_data.gyro)) {
				// TODO: Handle error
			}
		}
		// Check MAX_10S_GPS
		if (sensor_type & MAX_10S_GPS) {
			// Clear bits corresponding to this case
			ulTaskNotifyValueClear(Sample_Sensors_Handle, MAX_10S_GPS);
			parse_nmea(gps_data.gps_buffer);
		}
#endif
	}
  /* USER CODE END Sample_Sensors */
}

/* USER CODE BEGIN Header_LoRa_Radio */
/**
 * @brief Function implementing the LoRa_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LoRa_Radio */
void LoRa_Radio(void *argument)
{
  /* USER CODE BEGIN LoRa_Radio */
	/* Initialise the LoRa radio*/
	LoRa_reset(&LoRa_Handle);
	LoRa_setModulation(&LoRa_Handle, LORA_MODULATION);
	if (LoRa_init(&LoRa_Handle) != LORA_OK) {
		// Handle error
		printf("some error");
	}
	LoRa_startReceiving(&LoRa_Handle);

	/* Infinite loop */
	for (;;) {
		// Wait for LoRa to be ready before running task
		xTaskNotifyWait(0, 0, NULL, (TickType_t) portMAX_DELAY);
		uint8_t received_bytes = LoRa_received_bytes(&LoRa_Handle);
		uint8_t rf_buffer[RF_MAX_PACKET_SIZE] = { 0 };
		uint8_t bytes_read = LoRa_receive(&LoRa_Handle, rf_buffer, received_bytes);
		handle_rf_rx_packet(rf_buffer, (size_t) bytes_read);
	}
  /* USER CODE END LoRa_Radio */
}

/* USER CODE BEGIN Header_Sample_Baro */
/**
 * @brief Function implementing the Sample_Baro_Tas thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Sample_Baro */
void Sample_Baro(void *argument)
{
  /* USER CODE BEGIN Sample_Baro */
	/* Init MS5611 */
	if (!MS5611_init(&ms5611, osr)) {
		// TODO: Handle error
//		debug_print("[Main] MS5611 could not initialise\r\n",
//				sizeof("[Main] MS5611 could not initialise\r\n"), dbg = ERR);
		printf("Error");
	}
	/* Infinite loop */
	for (;;) {
#ifndef RUN_HITL
		// Read from baro
		ms5611_data.pressure = (float) MS5611_readPressure(&ms5611, 1);
		ms5611_data.altitude = (float) MS5611_getAltitude((double) ms5611_data.pressure, MS5611_BASELINE_PRESSURE);
		ms5611_data.temperature = (float) MS5611_readTemperature(&ms5611, 1);
#endif
		osDelay(1);
	}
  /* USER CODE END Sample_Baro */
}

/* USER CODE BEGIN Header_Data_Logging */
/**
 * @brief Function implementing the Data_Logging_Ta thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Data_Logging */
void Data_Logging(void *argument)
{
  /* USER CODE BEGIN Data_Logging */
	// Initialise SD card
	FRESULT res = SD_init();
	if (res != FR_OK) {
		// TODO: Handle error
		SD_card.flash_good = false;
		Error_Handler();
	}
	SD_card.flash_good = true;

	/* Infinite loop */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(1000.0 / (float )SD_card.log_frequency); // Number of ms to delay for
	xLastWakeTime = xTaskGetTickCount();
	for (;;) {
		osDelay(1000);
//	  vTaskDelayUntil(&xLastWakeTime, xFrequency);
		// TODO: Add logging to SD card
	}
  /* USER CODE END Data_Logging */
}

/* USER CODE BEGIN Header_GPS_Tracker */
/**
 * @brief Function implementing the GPS_Tracker_Tas thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_GPS_Tracker */
void GPS_Tracker(void *argument)
{
  /* USER CODE BEGIN GPS_Tracker */
	/* Infinite loop */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(1000.0 / (float )gps_tracker.chirp_frequency); // Number of ms to delay for
	xLastWakeTime = xTaskGetTickCount();
	// Reset GPS
	HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_RESET);
	osDelay(100);
	HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);

	for (;;) {
		if (gps_tracker.tracking_enabled) {
			vTaskDelayUntil(&xLastWakeTime, xFrequency);
			gps_tracking_packet gps_tracker_pkt = { .latitude = minmea_tocoord(&gps.gga_frame.latitude), .longitude = minmea_tocoord(&gps.gga_frame.longitude), .altitude = minmea_tofloat(&gps.gga_frame.altitude), .satellites_tracked = gps.gga_frame.satellites_tracked, };
			send_rf_packet(GPS_TRACKING_PACKET, (uint8_t*) &gps_tracker_pkt, sizeof(gps_tracker_pkt));


		} else {
			osDelay(1000);
		}

	}
  /* USER CODE END GPS_Tracker */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	// Timer elapsed therefore, GPS fix lost
	if (htim->Instance == TIM2) {
		gps.gps_good = false;
		TIM2->CNT = 0;
	}

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
