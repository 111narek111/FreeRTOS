/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

#define USART_DMA_BUFFER_SIZE   20
#define SPI_DMA_BUFFER_SIZE     20
//
uint8_t usart_rx_buffer[USART_DMA_BUFFER_SIZE];
uint8_t spi_rx_buffer[SPI_DMA_BUFFER_SIZE];

uint8_t spi_num_of_bytes;
uint8_t usart_num_of_bytes;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

USART_HandleTypeDef husart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

osThreadId usartToSpiHandle;
osThreadId spiToUsartHandle;
osThreadId lowPowerHandle;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void usart_to_spi(void const *argument);
void spi_to_usart(void const *argument);
void enterLowPower(void const *argument);
uint8_t reverse_bits(uint8_t number);

int main(void) {
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_Init();
	MX_SPI2_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(usartTOSpi, usart_to_spi, osPriorityAboveNormal, 0, 128);
	usartToSpiHandle = osThreadCreate(osThread(usartTOSpi), NULL);
	osThreadDef(spiToUsart, spi_to_usart, osPriorityAboveNormal, 0, 128);
	spiToUsartHandle = osThreadCreate(osThread(spiToUsart), NULL);
	osThreadDef(lowEnergy, enterLowPower, osPriorityNormal, 0, 128);
	lowPowerHandle = osThreadCreate(osThread(lowEnergy), NULL);

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	while (1) {

	}
}
//Task for entrance to low energy mode
void enterLowPower(void const *argument) {
	for (;;) {
		HAL_PWR_EnterSLEEPMode(0, PWR_SLEEPENTRY_WFI);
	}
}
//Task for USART message receiving and transmitting via SPI
void usart_to_spi(void const *argument) {
	uint8_t spi_tx[20];
	for (;;) {
		HAL_USART_Receive_DMA(&husart2, usart_rx_buffer, 1);
		ulTaskNotifyTake(0, portMAX_DELAY);
		for (int k = 0; k < spi_num_of_bytes; k++) {
			spi_tx[k] = reverse_bits(usart_rx_buffer[k]);
		}
		for (uint8_t j = 1; j < spi_num_of_bytes; j += 2) {
			uint8_t temp = spi_tx[j - 1];
			spi_tx[j - 1] = spi_tx[j];
			spi_tx[j] = temp;
		}
		HAL_SPI_Transmit_DMA(&hspi2, spi_tx, spi_num_of_bytes);
		spi_num_of_bytes = 0;
	}
}
//Task for SPI message receiving and transmitting via USART
void spi_to_usart(void const *argument) {
	uint8_t usart_tx[20];
	for (;;) {
		HAL_SPI_Receive_DMA(&hspi2, spi_rx_buffer, 1);
		ulTaskNotifyTake(0, portMAX_DELAY);
		for (uint8_t k = 0; k < usart_num_of_bytes; k++) {
			usart_tx[k] = reverse_bits(spi_rx_buffer[k]);
		}
		for (uint8_t j = 1; j < usart_num_of_bytes; j += 2) {
			uint8_t temp = usart_tx[j - 1];
			usart_tx[j - 1] = usart_tx[j];
			usart_tx[j] = temp;
		}
		HAL_USART_Transmit_DMA(&husart2, usart_tx, usart_num_of_bytes);
		usart_num_of_bytes = 0;
	}
}
//USART data receive via DMA callback
void HAL_USART_RxCpltCallback(USART_HandleTypeDef *husart) {
	HAL_TIM_Base_Stop_IT(&htim2);
	TIM2->CNT = 0;
	HAL_TIM_Base_Start_IT(&htim2);
	usart_num_of_bytes++;
	if (usart_num_of_bytes > 9) {
		vTaskNotifyGiveFromISR(usartToSpiHandle, 0);
	} else {
		HAL_USART_Receive_DMA(&husart2, &usart_rx_buffer[usart_num_of_bytes],
				1);
	}

}
//SPI data receive via DMA callback
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	HAL_TIM_Base_Stop_IT(&htim3);
	TIM3->CNT = 0;
	HAL_TIM_Base_Start_IT(&htim3);
	spi_num_of_bytes++;
	if (spi_num_of_bytes == 10) {
		vTaskNotifyGiveFromISR(spiToUsartHandle, 0);
	} else {
		HAL_SPI_Receive_DMA(&hspi2, &spi_rx_buffer[spi_num_of_bytes], 1);
	}

}
//timers timeout callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	} else if (htim->Instance == TIM2) {
		vTaskNotifyGiveFromISR(usartToSpiHandle, 0);
	} else if (htim->Instance == TIM3) {
		vTaskNotifyGiveFromISR(spiToUsartHandle, 0);
	}

}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
//timer for USART data receive timeout
static void MX_TIM2_Init(void) {
	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 834;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
//timer for SPI data receive timeout
static void MX_TIM3_Init(void) {
	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 584;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_Init(void) {
	husart2.Instance = USART2;
	husart2.Init.BaudRate = 115200;
	husart2.Init.WordLength = USART_WORDLENGTH_8B;
	husart2.Init.StopBits = USART_STOPBITS_1;
	husart2.Init.Parity = USART_PARITY_NONE;
	husart2.Init.Mode = USART_MODE_TX_RX;
	husart2.Init.CLKPolarity = USART_POLARITY_LOW;
	husart2.Init.CLKPhase = USART_PHASE_1EDGE;
	husart2.Init.CLKLastBit = USART_LASTBIT_DISABLE;
	if (HAL_USART_Init(&husart2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	/* DMA1_Channel6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	/* DMA1_Channel7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
}
/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {

	__disable_irq();
	NVIC_SystemReset();
	while (1) {
	}

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
//Function to change bit order in byte
uint8_t reverse_bits(uint8_t number) {
	uint8_t reversed_number = 0;
	for (uint8_t i = 0; i < 8; i++) {
		if (number & (1 << (7 - i))) {
			reversed_number |= (1 << i);
		}
	}
	return reversed_number;
}
