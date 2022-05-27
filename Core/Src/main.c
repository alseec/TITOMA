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
#include "strings.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define MAX_NUMBER_LEDs (6)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*Struct to encapsilate LEDs Port, Pin*/
typedef struct
{
  uint16_t     pin;
  GPIO_TypeDef *port;
}led_gpio_port_pin_t;


/*global instance of LED to use*/
led_gpio_port_pin_t led_gpio[MAX_NUMBER_LEDs] =
{
  {.port = LED_1_GPIO_Port, .pin = LED_1_Pin},
  {.port = LED_2_GPIO_Port, .pin = LED_2_Pin},
  {.port = LED_3_GPIO_Port, .pin = LED_3_Pin},
  {.port = LED_4_GPIO_Port, .pin = LED_4_Pin},
  {.port = LED_5_GPIO_Port, .pin = LED_5_Pin},
  {.port = LED_6_GPIO_Port, .pin = LED_6_Pin}
};


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t flag = 0;
int8_t cont = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t buffer[1];
uint32_t v_use = 0;

uint8_t uart_rx_data = 0;
uint32_t toggle_period = 0;
uint32_t led_tick = 0;
uint8_t toggle_led_enabled = 0;

const char *msg1 = "Msg demo Uart IT\r\n";
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//---------------------//

//--------- Start - Toggle ---------//
void toggle (uint8_t cont, uint32_t frec)
{
	static uint32_t tick = 0;
	if (tick < HAL_GetTick()){
		tick = HAL_GetTick() + frec;

		for (size_t i = 0; i < MAX_NUMBER_LEDs; i++)
			HAL_GPIO_TogglePin(led_gpio[i].port, led_gpio[i].pin);
		cont++;
	}
}
//--------- END - Toggle ---------//


//--------- Start - Sequence ---------//
void leds_sequence()
{
	if(flag == 0)
	{
		for (size_t i = 0; i < MAX_NUMBER_LEDs; i++)
			HAL_GPIO_WritePin(led_gpio[i].port, led_gpio[i].pin, GPIO_PIN_RESET);

	}

	if(flag == 1)
	{
		toggle(1, 1000);
	}

	if(flag == 2)
	{
		toggle(2, 500);
	}

	if(flag == 3)
	{
		toggle(3, 250);
	}
}
//---------End - Sequence ---------//



//---------------------//
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart1, ptr, len, 5);
	return len;
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */



  /*/Reference
  HAL_NVIC_EnableIRQ(USART1_IRQn); //enable an interrupt line
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_SetPendingIRQ(USART1_IRQn);

  HAL_NVIC_EnableIRQ(WWDG_IRQn); //enable an interrupt line
  HAL_NVIC_SetPriority(WWDG_IRQn, 2, 0);
  /*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //uint8_t counter = 0;


  while (1)
  {
//--------- Principal Funct ---------//
	  leds_sequence();
//--------------------------------//

    /* USER CODE END WHILE */

	  HAL_UART_Receive_IT(&huart1, buffer, 1);
	  HAL_NVIC_EnableIRQ(USART1_IRQn); 			// Enable the USART1 interrupt of the peripheral
	  HAL_NVIC_SetPendingIRQ(USART1_IRQn);		// the USART interrupt was setting in pending mode
	  HAL_NVIC_EnableIRQ(EXTI0_IRQn); 			// Enable the External interrupt (A0) of the GPIO peripheral
	  HAL_NVIC_SetPendingIRQ(EXTI0_IRQn);


/*/
	  printf("counter =  %d\r\n", counter);
	  HAL_Delay(2000);
	  counter++;
/*/

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}



/*/ Reference
void WWDG_IRQHandler(void)
{
	printf("In --> WWDG_IRQHandler\r\n");
	while(1);
}

void USART1_IRQHandler(void)
{
	printf("In --> USART1_IRQHandler\r\n");
	HAL_NVIC_SetPendingIRQ(WWDG_IRQn);

	while(1)
	{
		printf("In while --> USART1_IRQHandler\r\n");
		HAL_Delay(500);
	}
}
/*/

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
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LED_6_Pin|LED_5_Pin|LED_1_Pin|LED_2_Pin
                          |LED_3_Pin|LED_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_6_Pin LED_5_Pin LED_1_Pin LED_2_Pin
                           LED_3_Pin LED_4_Pin */
  GPIO_InitStruct.Pin = LED_6_Pin|LED_5_Pin|LED_1_Pin|LED_2_Pin
                          |LED_3_Pin|LED_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	printf("UART1 Transfer complete IT\r\n");
}


//--------- Start - External interrupt SERIAL) ---------//
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(buffer[0] == 's')
	{
		printf("------------|Key Pressed|------------\r\n");

		if (flag == 0){
			printf("----> State - LOW <----\r\n");
			flag++;
			return;
		}
		if(flag == 1)
		{
			printf("----> State - MEDIUM <----\r\n");
			flag++;
			return;
		}
		if(flag == 2)
		{
			printf("----> State - FAST <----\r\n");
			flag++;
			return;
		}
		if(flag == 3)
		{
			printf("----> State - OFF <----\r\n");
			flag = 0;
			return;
		}
		flag++;
	}
}
//--------- End - External interrupt (SERIAL) ---------//


//--------- Start - External interrupt (BUTTON) ---------//
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin == GPIO_PIN_0)
	{
		printf("------------| Push Button Pressed |------------\r\n");


		if(flag == 0)
		{
			printf("----> State - LOW <----\r\n");
			flag++;
			return;
		}
		if(flag == 1)
		{
			printf("----> State - MEDIUM <----\r\n");
			flag++;
			return;
		}
		if(flag == 2)
		{
			printf("----> State - FAST <----\r\n");
			flag++;
			return;
		}
		if(flag == 3)
		{
			printf("----> State - OFF <----\r\n");
			flag = 0;
			return;
		}
		flag++;
	}

}
//--------- End - External interrupt (BUTTON) ---------//
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
