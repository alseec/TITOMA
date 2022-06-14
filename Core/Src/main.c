/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Uart_Driver.h"
#include "BMP280_def.h"
#include "Command_Parser.h"

#include <stdio.h>

uart_driver_t uart_driver;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum Control_State{

	v_cold 	= 0,
	cold 	= 1,
	mild 	= 2,
	hot 	= 3,
	v_hot 	= 4,
	alarm 	= 5,
}Control_State;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define BUFF_SIZE 10000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/*/
const uint16_t dig_T1 = 27504;
const int16_t dig_T2 = 26435;
const int16_t dig_T3 = -1000;

double temperature_raw = 0;
/*/
float temperature_in_degrees = 0;
float Temperature_to_show = 0;



uint16_t toggle_period = 1000;
uint32_t led_tick = 0;
int8_t cont_door = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 10);

	//Usar send
	//uart_driver_send(&uart_driver, (uint8_t *)ptr, len);

	return len;
}



//---------------START - DOOR with INTERRUPT (In use)----------------------//
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin == GPIO_PIN_0){
		printf("------------| USER Button Pressed |------------\r\n");
		cont_door++;

		if(cont_door == 1){
			HAL_GPIO_WritePin(DOOR_GPIO_Port, DOOR_Pin, GPIO_PIN_SET);
			printf("------------| OPENING DOOR |------------\r\n");
			cont_door = -1;
		}

		if(cont_door == HAL_OK){
			HAL_GPIO_WritePin(DOOR_GPIO_Port, DOOR_Pin, GPIO_PIN_RESET);
			printf("------------| CLOSING DOOR |------------\r\n");
		}
	}
}
//---------------END - DOOR with INTERRUPT (In use)----------------------//

//---------------START - PWM-ADC Control ----------------------//



uint8_t control_pwm (uint8_t mode)
{
	uint8_t state = 0;


	if (mode == 0){
		state = 000.0;
		/*/
		uint16_t pwm_dma_buff[BUFF_SIZE] = {0};
		for (uint16_t index = 0; index < BUFF_SIZE; index++){
			if (index < 0){
				pwm_dma_buff[index] = 0;
			}
		HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwm_dma_buff, BUFF_SIZE);
		//FAN 0%
		}
		/*/
	}


	if (mode == 25){

		state = 025.0;

		uint16_t pwm_dma_buff[BUFF_SIZE] = {0};
		for (uint16_t index = 100; index < BUFF_SIZE; index++){
			if (index < 300){
				pwm_dma_buff[index] = 10;
			}else if (index < 500){
				pwm_dma_buff[index] = 50;
			}else if (index < 800){
				pwm_dma_buff[index] = 100;
			}else{
				pwm_dma_buff[index] = 250;
			}
		}
		HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwm_dma_buff, BUFF_SIZE);
		//FAN 25%
	}

	if (mode == 50){

		state = 50.0;

		uint16_t pwm_dma_buff[BUFF_SIZE] = {0};
		for (uint16_t index = 100; index < BUFF_SIZE; index++){
			if (index < 300){
				pwm_dma_buff[index] = 10;
			}else if (index < 500){
				pwm_dma_buff[index] = 100;
			}else if (index < 800){
				pwm_dma_buff[index] = 250;
			}else{
				pwm_dma_buff[index] = 500;
			}
		}
		HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwm_dma_buff, BUFF_SIZE);
		//FAN 50%
	}

	if (mode == 75){

		state = 75.0;
		uint16_t pwm_dma_buff[BUFF_SIZE] = {0};
		for (uint16_t index = 100; index < BUFF_SIZE; index++){
			if (index < 300){
				pwm_dma_buff[index] = 10;
			}else if (index < 500){
				pwm_dma_buff[index] = 150;
			}else if (index < 800){
				pwm_dma_buff[index] = 350;
			}else{
				pwm_dma_buff[index] = 750;
			}
		}
		HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwm_dma_buff, BUFF_SIZE);
		//FAN 75%
	}

	if (mode == 100){
		state = 100.0;
		mode = 0;

		uint16_t pwm_dma_buff[BUFF_SIZE] = {0};
		for (uint16_t index = 100; index < BUFF_SIZE; index++){
			if (index < 300){
				pwm_dma_buff[index] = 10;
			}else if (index < 500){
				pwm_dma_buff[index] = 200;
			}else if (index < 700){
				pwm_dma_buff[index] = 500;
			}else if (index < 900){
				pwm_dma_buff[index] = 700;
			}else{
				pwm_dma_buff[index] = 1000;
			}
		}
		HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwm_dma_buff, BUFF_SIZE);
		//FAN 100%
	}
	return state;
}

/*/
void State_func(uint8_t mode){


	if (mode == v_cold){

		printf("------------------------ \r\n");
		printf("------------| Very Cold |------------ \r\n");
		printf("------------------------ \r\n");

		HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(HEATER_GPIO_Port, HEATER_Pin, GPIO_PIN_SET);

		printf ("ALARM -> OFF\r\nCOOLER -> OFF\r\nHEATER -> ON\r\n");
		printf("------------------------ \r\n");

	}

	if (mode == cold){

		printf("------------------------ \r\n");
		printf("------------| Cold |------------ \r\n");
		printf("------------------------ \r\n");

		HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(HEATER_GPIO_Port, HEATER_Pin, GPIO_PIN_RESET);

		printf ("ALARM -> OFF\r\nCOOLER -> OFF\r\nHEATER -> OFF\r\n");
		printf("------------------------ \r\n");

	}

	if (mode == mild){

		printf("------------------------ \r\n");
		printf("------------| Mild |------------ \r\n");
		printf("------------------------ \r\n");

		HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(HEATER_GPIO_Port, HEATER_Pin, GPIO_PIN_RESET);

		uint16_t pwm_dma_buff[BUFF_SIZE] = {0};
		for (uint16_t index = 100; index < BUFF_SIZE; index++){
			if (index < 300){
				pwm_dma_buff[index] = 10;
			}else if (index < 500){
				pwm_dma_buff[index] = 100;
			}else{
				pwm_dma_buff[index] = 200;
			}
		}
		HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwm_dma_buff, BUFF_SIZE);

		printf ("ALARM -> OFF\r\nCOOLER -> ON |20|\r\nHEATER -> OFF\r\n");
		printf("------------------------ \r\n");
		//FAN 20%

	}

	if (mode == hot){

		printf("------------------------ \r\n");
		printf("------------| Hot |------------ \r\n");
		printf("------------------------ \r\n");

		HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(HEATER_GPIO_Port, HEATER_Pin, GPIO_PIN_RESET);

		uint16_t pwm_dma_buff[BUFF_SIZE] = {0};
		for (uint16_t index = 100; index < BUFF_SIZE; index++){
			if (index < 300){
				pwm_dma_buff[index] = 10;
			}else if (index < 500){
				pwm_dma_buff[index] = 100;
			}else if (index < 800){
				pwm_dma_buff[index] = 300;
			}else{
				pwm_dma_buff[index] = 600;
			}
		}
		HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwm_dma_buff, BUFF_SIZE);

		printf ("ALARM -> OFF\r\nCOOLER -> ON |60|\r\nHEATER -> OFF\r\n");
		printf("------------------------ \r\n");
		//FAN 60%

	}

	if (mode == v_hot){

		printf("------------------------ \r\n");
		printf("------------| Very Hot |------------ \r\n");
		printf("------------------------ \r\n");

		HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(HEATER_GPIO_Port, HEATER_Pin, GPIO_PIN_RESET);

		uint16_t pwm_dma_buff[BUFF_SIZE] = {0};
		for (uint16_t index = 100; index < BUFF_SIZE; index++){
			if (index < 300){
				pwm_dma_buff[index] = 10;
			}else if (index < 500){
				pwm_dma_buff[index] = 100;
			}else if (index < 800){
				pwm_dma_buff[index] = 500;
			}else{
				pwm_dma_buff[index] = 1000;
			}
		}
		HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwm_dma_buff, BUFF_SIZE);

		printf ("ALARM -> OFF\r\nCOOLER -> ON |100|\r\nHEATER -> OFF\r\n");
		printf("------------------------ \r\n");
		//FAN 100%

	}

	if (mode == alarm){

		printf("------------------------ \r\n");
		printf("------------| ALARM!! |------------ \r\n");
		printf("------------------------ \r\n");

		HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(HEATER_GPIO_Port, HEATER_Pin, GPIO_PIN_RESET);

		printf ("ALARM -> ON\r\nCOOLER -> OFF\r\nHEATER -> OFF\r\n");
		printf("------------------------ \r\n");

	}
}
/*/

/*/
uint8_t cooler_state(uint8_t mode)
{
	uint8_t fan_st = -1;

	switch (mode){

	case v_cold:
		fan_st = 000;
		break;
	case cold:
		fan_st = 000;
		break;
	case mild:
		fan_st = 020;
		break;
	case hot:
		fan_st = 060;
		break;
	case v_hot:
		fan_st = 100;
		break;
	case alarm:
		fan_st = 000;
		break;
	default:
		break;

	}

}
/*/


//---------------END - PWM-ADC Control ----------------------//
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
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  uart_driver_init(&uart_driver, &huart1);

  //uint8_t startup_message[] = "Hello World\r\n";
  //uart_driver_send(&uart_driver, startup_message, sizeof(startup_message) - 1);


  //---------------START - Manual Sensor Configuration ----------------------//

  /*/
  uint8_t spi_tx_data = BMP280_REG_ADDR_ID;
  uint8_t spi_rx_data[3] = {0x00};

  HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&hspi1, &spi_tx_data, 1, 100) == HAL_OK){
	  if (HAL_SPI_Receive(&hspi1, spi_rx_data, 1, 100) == HAL_OK){
		  HAL_Delay(100);
	  }
  }
  HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_Pin, GPIO_PIN_SET);

  spi_tx_data = BMP280_REG_ADDR_CTRL_MEAS - BMP280_REG_ADDR_RESET_MEAS;
  uint8_t spi_ctrl_rx_data[3] = {0x00};

  HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&hspi1, &spi_tx_data, 1, 100)==HAL_OK){
	  spi_tx_data = 0b10000011 ;
	  if (HAL_SPI_Transmit(&hspi1, &spi_tx_data, 1, 100)==HAL_OK){
		  if (HAL_SPI_Receive(&hspi1, spi_ctrl_rx_data, 1, 100)== HAL_OK){
		  }
	  }
  }
  HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_Pin, GPIO_PIN_SET);
  /*/
  //---------------START - Manual Sensor Configuration ----------------------//


//-----------------------//
  Init_Sensor();


//-----------------------//

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  uart_driver_run(&uart_driver);

	  HAL_GetTick();

	  if (HAL_GetTick() - led_tick > toggle_period){
		  led_tick = HAL_GetTick();
		  Measure();

	  }

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, COOLER_Pin|HEATER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, DOOR_Pin|HEARTBEAT_Pin|ALARM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BMP280_CS_Pin */
  GPIO_InitStruct.Pin = BMP280_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BMP280_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : COOLER_Pin HEATER_Pin */
  GPIO_InitStruct.Pin = COOLER_Pin|HEATER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : DOOR_Pin HEARTBEAT_Pin ALARM_Pin */
  GPIO_InitStruct.Pin = DOOR_Pin|HEARTBEAT_Pin|ALARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
