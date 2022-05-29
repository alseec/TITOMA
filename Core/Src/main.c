	/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum BMP280_Register_Address{
	BMP280_REG_ADDR_ERROR			 = 0x00,
	BMP280_REG_ADDR_CAL_START_ADDR	 = 0x88,
	BMP280_REG_ADDR_ID				 = 0xD0,
	BMP280_REG_ADDR_RESET			 = 0xE0,
	BMP280_REG_ADDR_STATUS			 = 0xF3,
	BMP280_REG_ADDR_CTRL_MEAS		 = 0xF4,
	BMP280_REG_ADDR_CONFIG			 = 0xF5,
	BMP280_REG_ADDR_PRESS_MSB	 	 = 0xF7,
	BMP280_REG_ADDR_PRESS_LSB		 = 0xF8,
	BMP280_REG_ADDR_PRESS_XLSB		 = 0xF9,
	BMP280_REG_ADDR_TEMP_MSB		 = 0xFA,
	BMP280_REG_ADDR_TEMP_LSB		 = 0xFB,
	BMP280_REG_ADDR_TEMP_XLSB		 = 0xFC,
	BMP280_REG_ADDR_RESET_V			 = 0xB6,
	BMP280_REG_ADDR_RESET_MEAS		 = 0x80,
}BMP280_Register_Address;


typedef struct BMP280_Register_Info{
	uint16_t 	dig_T1;
	int16_t		dig_T2;
	int16_t 	dig_T3;
	uint16_t 	dig_P1;
	int16_t 	dig_P2;
	int16_t 	dig_P3;
	int16_t 	dig_P4;
	int16_t 	dig_P5;
	int16_t 	dig_P6;
	int16_t 	dig_P7;
	int16_t 	dig_P8;
	int16_t 	dig_P9;
}BMP280_Register_Info;


typedef enum BMP280_Power_Mode{
	Sleep_Mode 	= 0b00,
	Forced_Mode = 0b01,
	Normal_Mode = 0b11,
	//0xF4 [1:0]
}BMP280_Power_Mode;

typedef enum BMP280_IIR_Filter {
	Filter_off 		= 0b000,
	Filter_coeff_2 	= 0b001,
	Filter_coeff_4 	= 0b010,
	Filter_coeff_8 	= 0b011,
	Filter_coeff_16 = 0b100,
	//0xF5 [2:0]
}BMP280_IIR_Filter;

typedef enum BMP280_OverS{
	Oversampling_WO		= 0b000,
	Oversamplingx1 		= 0b001,
	Oversamplingx2 		= 0b010,
	Oversamplingx4 		= 0b011,
	Oversamplingx8 		= 0b100,
	Oversamplingx16 	= 0b101,
}BMP280_OverS;

typedef enum BMP280_StandbyTime{
	standby_time_500us = 0b000,
	standby_time_62500us = 0b001,
	standby_time_125ms = 0b010,
	standby_time_250ms = 0b011,
	standby_time_500ms = 0b100,
	standby_time_1000ms = 0b101,
	standby_time_2000ms = 0b110,
	standby_time_4000ms = 0b111
}BMP280_StandbyTime;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP280_ID								(0x58)
//--------------------------//
#define BMP280_SPI_WRITING_MASK                 (0x7F) // = 0b01111111
#define BMP280_SPI_READING_MASK                 (0xFF) // = 0b11111111
//--------------------------//
#define FILTER_MASK								0b11100011


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

int32_t t_fine;
const uint16_t dig_T1 = 27504;
const int16_t dig_T2 = 26435;
const int16_t dig_T3 = -1000;

double temperature_raw = 0;
float temperature_in_degrees = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//---------------START - CS Configurations----------------------//

void CS_Config_Low() {

		HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_Pin, GPIO_PIN_RESET);
}


void CS_Config_High() {
	HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_Pin, GPIO_PIN_SET);
}
//---------------END - CS Configurations----------------------//

//---------------START - READ/WRITE FUNCTIONS----------------------//
uint8_t SPI_Read_Write(uint8_t spi_tx_data)
{
	uint8_t spi_rx_data = 255;
	if (HAL_SPI_Transmit(&hspi1, &spi_tx_data, 1, 100) == HAL_OK){
		if (HAL_SPI_Receive(&hspi1, &spi_rx_data, 1, 100) == HAL_OK){
			return spi_rx_data;
		}
	}
	return 0;
}


uint8_t Read_Register (uint8_t Register_Address) {

	CS_Config_Low();
	SPI_Read_Write(Register_Address & BMP280_SPI_READING_MASK);
	uint8_t value = SPI_Read_Write(0);
	CS_Config_High();
	return value;
}


void Write_Register(uint8_t Register_Address, uint8_t value)
{
	CS_Config_Low();
	SPI_Read_Write(Register_Address & BMP280_SPI_WRITING_MASK);
	SPI_Read_Write(value);
	CS_Config_High();
}
//---------------END - READ/WRITE FUNCTIONS----------------------//

//---------------START - CONFIG SENSOR FUNCTIONS----------------------//

void Filter_IIR(BMP280_IIR_Filter data_f){

	uint8_t conf = Read_Register(BMP280_REG_ADDR_CONFIG);
	conf = (conf & 0b11100011) | (data_f << 2); // off (000)
	Write_Register(BMP280_REG_ADDR_CONFIG, conf);
}

void Power_Mode(BMP280_Power_Mode mode)
{
	uint8_t value = Read_Register(BMP280_REG_ADDR_CTRL_MEAS);
	value = (value & 0b11111100) | mode; //Sleep_mode (00)
	Write_Register(BMP280_REG_ADDR_CTRL_MEAS, value);
}

void Temperature_OS(BMP280_OverS osrs_t)
{
	uint8_t conf = Read_Register(BMP280_REG_ADDR_CTRL_MEAS);
	conf = (conf & 0b00011111) | (osrs_t << 5);
	Write_Register(BMP280_REG_ADDR_CTRL_MEAS, conf);
}

void StandbyTime(BMP280_StandbyTime t_sb)
{
	uint8_t conf = Read_Register(BMP280_REG_ADDR_CONFIG);
	conf = (conf & 0b00011111) | (t_sb << 5);
	Write_Register(BMP280_REG_ADDR_CONFIG, conf);
}
//---------------END - CONFIG SENSOR FUNCTIONS----------------------//

//---------------START - SENSOR COMMANDS----------------------//
uint8_t getID()
{
	return Read_Register(BMP280_REG_ADDR_ID);
}

void Reset()
{
	Write_Register(BMP280_REG_ADDR_CAL_START_ADDR, BMP280_REG_ADDR_RESET_V);
}
//---------------START - SENSOR COMMANDS----------------------//


//---------------START - TEMPERATURE CONFIG----------------------//
float BMP280_Convert_Formula (double mea_temp){
	double var1 = 0, var2 = 0;
	float T = 0;

	var1 = (((double)mea_temp) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
	var2 = ((((double)mea_temp) / 131072.0 - ((double)dig_T1) / 8192.0) * (((double)mea_temp)/131072.0 - ((double)dig_T1) / 8192.0)) * ((double)dig_T3);
	T = (float)(var1 + var2) / 5120.0;
	return T;
}


void Measure()
{
	uint8_t tx_data = BMP280_REG_ADDR_TEMP_MSB;
	uint8_t rx_data[3] = {0x00};


	HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_Pin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(&hspi1, &tx_data, 1, 100)==HAL_OK){
		if (HAL_SPI_Receive(&hspi1, rx_data, 3, 100)== HAL_OK){

			int32_t adc_T = (int32_t)(rx_data[2] & 0b00001111) | (rx_data[1] << 4) | (rx_data[0] << 12);
			temperature_in_degrees = (float)(BMP280_Convert_Formula(adc_T));
		}
	}
	HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_Pin, GPIO_PIN_SET);
}

//---------------END - TEMPERATURE CONFIG----------------------//

//---------------START - PRINCIPAL INIT----------------------//
uint8_t Init_Sensor()
{
	if (getID() != BMP280_ID)
	{
		return 1;
	}
	//Reset();

	//------ Start Config sensor ----//
	Temperature_OS(Oversamplingx2);

	Power_Mode(Normal_Mode);
	Filter_IIR(Filter_coeff_16);
	StandbyTime(standby_time_1000ms);
	//----- End Config sensor-----//

	return 0;
}
//---------------END - PRINCIPAL INIT----------------------//


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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

//Config without function
/*/
  uint8_t spi_tx_data = BMP280_REG_ADDR_ID;
  uint8_t spi_rx_data[3] = {0x00};

  HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&hspi1, &spi_tx_data, 1, 100) == HAL_OK){
	  if (HAL_SPI_Receive(&hspi1, spi_rx_data, 1, 100) == HAL_OK){
		  HAL_Delay(100);
	  }
  }
  //HAL_Delay(100);
/*/

  uint8_t spi_tx_data = BMP280_REG_ADDR_CTRL_MEAS - BMP280_REG_ADDR_RESET_MEAS;
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
  //HAL_Delay(100);

/*/
  spi_tx_data = BMP280_REG_ADDR_CONFIG;
  //spi_ctrl_rx_data[3] = {0x00};
  //uint8_t ctr_rx_data[2] = {BMP280_REG_ADDR_CTRL_MEAS, meas};


  HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&hspi1, &spi_tx_data, 1, 100)==HAL_OK){
	  spi_config_tx_data = 0b10110101 ;
	  if (HAL_SPI_Transmit(&hspi1, &spi_config_tx_data, 1, 100)==HAL_OK){
		  if (HAL_SPI_Receive(&hspi1, spi_ctrl_rx_data, 1, 100)== HAL_OK){
		  }
	  }
  }
  HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_Pin, GPIO_PIN_SET);
/*/
//
//


  //------------------------------//
/*/ Reference
  uint8_t startup_message[] = "Hello TITOMA\r\n";
  size_t startup_message_size = sizeof(startup_message) - 1;

  uint8_t uart_rx_data = 0;
  uint32_t toggle_period = 0;
  uint32_t led_tick = 0;
  uint8_t toggle_led_enabled = 0;

  HAL_UART_Transmit(&huart1, startup_message, startup_message_size, 500);
/*/
  //------------------------------//

  Init_Sensor();
  //Measure();

  //------------------------------//
  uint8_t uart_rx_data = 0;
  uint32_t toggle_period = 1000;
  uint32_t led_tick = 0;
  uint8_t toggle_led_enabled = 0;


  uint8_t noID_msg[] = "Sensor missing\r\n";
  size_t noID_msg_size = sizeof(noID_msg) - 1;

  uint8_t Off_msg[] = "--- |OFF reading| ---\r\n";
  size_t Off_msg_size= sizeof(Off_msg)-1;

  uint8_t On_msg[] = "--- |ON reading| ---\r\n";
  size_t On_msg_size= sizeof(On_msg)-1;

  //------------------------------//

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (Init_Sensor() == HAL_OK){
		  HAL_UART_Transmit(&huart1, noID_msg, noID_msg_size, 500);
	  }


	  if (temperature_in_degrees > 40){
		  HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
	  }

	  if (HAL_UART_Receive(&huart1, &uart_rx_data, 1, 500) == HAL_OK){
		  HAL_UART_Transmit(&huart1, &uart_rx_data, 1, 50);

		  switch (uart_rx_data){
		  case 'E':
			  HAL_UART_Transmit(&huart1, Off_msg, Off_msg_size, 500);
			  toggle_led_enabled = 0;
			  break;
		  case 'F':
			  HAL_UART_Transmit(&huart1, On_msg, On_msg_size, 500);
			  toggle_led_enabled = 1;
			  break;
		  default:
			  break;
		  }
	  }

	  if (toggle_led_enabled){
		  if (HAL_GetTick() - led_tick > toggle_period){
			  led_tick = HAL_GetTick();
			  Measure();
		  }
	  }


//	  while (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) != GPIO_PIN_SET);
//	  while (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) != GPIO_PIN_RESET);

//	  HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);

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
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BMP280_CS_Pin */
  GPIO_InitStruct.Pin = BMP280_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BMP280_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GREEN_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
