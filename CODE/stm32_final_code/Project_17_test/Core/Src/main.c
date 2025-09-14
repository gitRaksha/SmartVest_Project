/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.</center></h2>
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lora.h"
#include "TJ_MPU6050.h"
#include <stdio.h>
#include <string.h>
#include "core_cm4.h"  // For CoreDebug and ITM registers
#include <math.h>
#include "stm32f4xx_hal.h"
#define MPU6050_ADDRESS 0x68
#define MPU6050_WHO_AM_I 0x75
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MQ7_ADC_PIN          GPIO_PIN_0     // e.g., PA0 for ADC1
#define MQ7_ADC_PORT         GPIOA
#define MQ7_ADC_HANDLE       hadc1          // Your ADC handle
#define PWM_TIM_HANDLE       htim2          // e.g., TIM2 for PWM
#define PWM_TIM_CHANNEL      TIM_CHANNEL_1  // e.g., PA5 for TIM2_CH1
#define RL                   5.0f      // Load resistor (in ohms)

#define HIGH_HEAT_DURATION   60000  // 60s @ 5V
#define LOW_HEAT_DURATION    90000  // 90s @ 1.4V
#define RATIO_CLEAN_AIR      27.5f   // From datasheet
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
LoRa myLoRa;
uint16_t Lora_stat=0;
uint8_t RxBuffer[128];
RawData_Def myAccelRaw;
ScaledData_Def myAccelScaled;
MPU_ConfigTypeDef myMpuConfig;
char swv_buffer[128];
char msg[64];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t MPU6050_TestConnection(void);
void MQ7_Calibrate();
float MQ7_ReadPPM(float Rs_R0);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t fall_alert_flag = 0;
float latest_acc_magnitude = 0.0f;
float latest_ppm = 0.0f;

uint32_t last_lora_update = 0;
const uint32_t LORA_SEND_INTERVAL = 3000; // Send every 3s


uint32_t last_mpu_update = 0;
uint32_t last_mq7_update = 0;

uint32_t mq7_phase_start = 0;
uint8_t mq7_heating_phase = 1; // 1 = HIGH, 0 = LOW

const uint32_t MQ7_HIGH_HEAT_DURATION = 60000;
const uint32_t MQ7_LOW_HEAT_DURATION = 90000;

float MQ7_R0 = 10.0f; // calibrated in clean air
float MQ7_ReadPPM(float voltage) {
    float Rs = (5.0f - voltage) / voltage * RL;  // Compute sensor resistance
    float ratio = Rs / MQ7_R0;
    return 99.042f * powf(ratio, -1.518f);  // PPM = a*(Rs/Ro)^b (datasheet curve)
}

// SWV ITM Printf Redirection
int __io_putchar(int ch) {
    ITM_SendChar(ch);
    return ch;
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
// Lora part


	 myLoRa = newLoRa();

	 myLoRa.CS_port         = NSS_GPIO_Port;
	 myLoRa.CS_pin          = NSS_Pin;
	 myLoRa.reset_port      = RST_GPIO_Port;
	 myLoRa.reset_pin       = RST_Pin;
	 myLoRa.DIO0_port       = DIO0_GPIO_Port;
	 myLoRa.DIO0_pin        = DIO0_Pin;
	 myLoRa.hSPIx           = &hspi1;

	 myLoRa.frequency             = 433;             // default = 433 MHz
	 myLoRa.spredingFactor        = SF_7;            // default = SF_7
	 myLoRa.bandWidth = BW_125KHz; // pour correspondre à 125E3 sur Arduino	 myLoRa.crcRate               = CR_4_5;          // default = CR_4_5
	 myLoRa.power                 = POWER_17db;      // default = 20db
	 myLoRa.overCurrentProtection = 125;             // default = 100 mA
	 myLoRa.preamble              = 9;              // default = 8;
	 myLoRa.syncWord              = 0x34;         // à ajouter si supporté
	 myLoRa.crcRate = CR_4_5;          // Coding Rate 4/5

	 if (LoRa_init(&myLoRa)==LORA_OK) {
	     Lora_stat=1;
	     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	     printf("LoRa initialized!\r\n");
	     // Add version check here:
	     uint8_t version = LoRa_read(&myLoRa, RegVersion);
	     printf("LoRa Version: 0x%02X\n", version);  // Should be 0x12
	     if (version != 0x12) {
	         printf("LoRa Failed!!!\r\n");
	         Error_Handler();
	     } else {
	         printf("LoRa Version OK\r\n");
	     }

	 }

//MPU6050 Part
  // Initialize MPU6050
  MPU6050_Init(&hi2c1);

  // Test connection
  if(MPU6050_TestConnection() != HAL_OK) {
    printf("MPU6050   Connection Failed!\r\n");
    Error_Handler();
  }

  // Configure MPU6050
  myMpuConfig.Accel_Full_Scale = AFS_SEL_4g;
  myMpuConfig.ClockSource = Internal_8MHz;
  myMpuConfig.CONFIG_DLPF = DLPF_184A_188G_Hz;
  myMpuConfig.Sleep_Mode_Bit = 0;
  MPU6050_Config(&myMpuConfig);



  printf("MPU6050 Initialized Successfully!\r\n");
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

// MQ-7 part

  HAL_ADC_Start(&hadc1); // or whatever ADC you're using
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // Start TIM2 Channel 1 PWM


  printf("System Initialized\r\n");  // Test message
  printf("MQ-7 Sensor Monitoring Started\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t now = HAL_GetTick();

	      // === MPU6050 Handling ===
	      if (now - last_mpu_update >= 1000)
	      {
	          last_mpu_update = now;

	          MPU6050_Get_Accel_Scale(&myAccelScaled);

	          float acc_x = myAccelScaled.x;
	          float acc_y = myAccelScaled.y;
	          float acc_z = myAccelScaled.z;

	          latest_acc_magnitude = sqrtf(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
	          printf("acc_magnitude = %.2f\r\n", latest_acc_magnitude);

	          static uint8_t free_fall_detected = 0;
	          static uint8_t impact_detected = 0;

	          if (latest_acc_magnitude < 1200.0f) {
	              free_fall_detected = 1;
	          }
	          if (latest_acc_magnitude > 1500.0f) {
	              impact_detected = 1;
	          }

	          if (free_fall_detected && impact_detected) {
	              printf("⚠  FALL DETECTED!\r\n");

	              free_fall_detected = 0;
	              impact_detected = 0;
	              fall_alert_flag = 1; // Flag for LoRa
	          }

	          snprintf(swv_buffer, sizeof(swv_buffer),
	                   "Accel: X=%7.2f Y=%7.2f Z=%7.2f\r\n"
	                   "--------------------------------\r\n",
	                   acc_x, acc_y, acc_z);
	          printf("%s", swv_buffer);

	          HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13); // Activity LED
	      }

	      // === MQ7 Sensor Handling ===
	      if (now - last_mq7_update >= 1000)
	      {
	          last_mq7_update = now;

	          HAL_ADC_Start(&MQ7_ADC_HANDLE);
	          HAL_ADC_PollForConversion(&MQ7_ADC_HANDLE, HAL_MAX_DELAY);
	          uint32_t adcValue = HAL_ADC_GetValue(&MQ7_ADC_HANDLE);
	          float voltage = adcValue * 3.3f / 4095.0f;
	          latest_ppm = MQ7_ReadPPM(voltage);

	          printf("ADC: %lu | Voltage: %.2f V | CO: %.2f PPM\r\n", adcValue, voltage, latest_ppm);
	      }

	      // === MQ7 Heater Control ===
	      if (mq7_heating_phase && (now - mq7_phase_start >= MQ7_HIGH_HEAT_DURATION))
	      {
	          mq7_phase_start = now;
	          mq7_heating_phase = 0;
	          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 70);  // Simulate 1.4V phase
	      }
	      else if (!mq7_heating_phase && (now - mq7_phase_start >= MQ7_LOW_HEAT_DURATION))
	      {
	          mq7_phase_start = now;
	          mq7_heating_phase = 1;
	          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 255); // Simulate 5V phase
	      }

	      // === LoRa Transmission only when needed ===
	      if (fall_alert_flag || latest_ppm > 10.0f)
	      {
	          LoRa_gotoMode(&myLoRa, STNBY_MODE);
	          HAL_Delay(50);

	          char lora_message[100];
	          snprintf(lora_message, sizeof(lora_message),
	                   "{ \"fall\":%d, \"co\":%.2f, \"acc\":%.2f }",
	                   fall_alert_flag, latest_ppm, latest_acc_magnitude);

	          printf("Sending LoRa: %s\r\n", lora_message);

	          if (LoRa_transmit(&myLoRa, (uint8_t*)lora_message, strlen(lora_message), 2000))
	          {
	              printf("LoRa transmission OK\r\n");
	              HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // LED feedback
	          }

	          fall_alert_flag = 0; // Reset flag
	          HAL_Delay(3000); // Optional: delay to avoid flooding
	      }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_MPU_Pin|LED_MQ7_Pin|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : RST_Pin */
  GPIO_InitStruct.Pin = RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NSS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_MPU_Pin LED_MQ7_Pin PD14 PD15 */
  GPIO_InitStruct.Pin = LED_MPU_Pin|LED_MQ7_Pin|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t MPU6050_TestConnection(void)
{
  uint8_t who_am_i = 0;
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS << 1,
                                             MPU6050_WHO_AM_I, 1, &who_am_i, 1, 100);
  if(status == HAL_OK && who_am_i == 0x68) {
    return HAL_OK;
  }
  return HAL_ERROR;
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  for(int i = 0; i < len; i++) {
    ITM_SendChar(*ptr++);
  }
  return len;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
