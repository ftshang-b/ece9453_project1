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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "aes.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
uint64_t des(uint64_t input, uint64_t key, char mode);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t data_buffer[17] = {0};
  uint8_t xor_keys[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
  uint8_t des_keys[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
  uint8_t aes_keys[16] = {0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00};
  while (1)
  {
	  // 1. Receives data via the UART peripheral (format: 8N1)
	  // Machine receives data in the following format:
	  // <Protocol: 1 byte><Data: 16 bytes> - MSByte -> LSByte
	  HAL_UART_Receive(&huart1, data_buffer, 17, HAL_MAX_DELAY);

	  uint8_t protocol_byte = data_buffer[0];
	  bool unknownProtocol = false;

	  if (protocol_byte == 0x10) {
		  // No encryption
	  } else if (protocol_byte == 0x20) {
		  // XOR encryption
		  for (int i = 1; i < 17; i += 1)
		  {
			  data_buffer[i] = xor_keys[i - 1] ^ data_buffer[i];
		  }
	  } else if (protocol_byte == 0x30) {
		  // DES encryption
		  // Since data is 16 bytes long, will have to run DES decryption twice.
		  // Convert key to uint64_t
		  uint64_t key = 0;
		  for (int i = 0; i < 8; i += 1)
		  {
			  uint64_t current_byte = des_keys[i];
			  current_byte = current_byte << (64 - ((i + 1) * 8));
			  key = key | current_byte;
		  }

		  // Copy over first 8 bytes for decryption #1
		  uint8_t first_8_bytes[8] = {0};
		  for (int i = 0; i < 8; i += 1)
		  {
			  first_8_bytes[i] = data_buffer[i + 1];
		  }

		  // convert to uint64_t for DES implementation
		  uint64_t first_ciphertext = 0;
		  for (int i = 0; i < 8; i += 1)
		  {
			uint64_t current_byte = first_8_bytes[i];
			current_byte = current_byte << (64 - ((i + 1) * 8));
			first_ciphertext = first_ciphertext | current_byte;
		  }

		  // Copy over second 8 bytes for decryption #2
		  uint8_t second_8_bytes[8] = {0};
		  for (int i = 0; i < 8; i += 1)
		  {
			  second_8_bytes[i] = data_buffer[i + 1 + 8];
		  }

		  // convert to uint64_t for DES implementation
		  uint64_t second_ciphertext = 0;
		  for (int i = 0; i < 8; i += 1)
		  {
			uint64_t current_byte = second_8_bytes[i];
			current_byte = current_byte << (64 - ((i + 1) * 8));
			second_ciphertext = second_ciphertext | current_byte;
		  }

		  // Run DES decryption for both ciphertexts.
		  uint64_t first_result = des(first_ciphertext, key, 'd');
		  uint64_t second_result = des(second_ciphertext, key, 'd');

		  // convert first_result and second_result back to uint8 array
		  for (int i = 0; i < 8; i += 1)
		  {
			  uint64_t shifted_first = first_result >> (64 - (8 * (i + 1)));
			  uint64_t shifted_second = second_result >> (64 - (8 * (i + 1)));
			  uint64_t mask = 0xFF;
			  uint8_t result_first = shifted_first & mask;
			  uint8_t result_second = shifted_second & mask;
			  first_8_bytes[i] = result_first;
			  second_8_bytes[i] = result_second;
		  }

		  // Place decrypted data back into data buffer.
		  for (int i = 1; i < 9; i += 1)
		  {
			  data_buffer[i] = first_8_bytes[i - 1];
			  data_buffer[i + 8] = second_8_bytes[i - 1];
		  }

	  } else if (protocol_byte == 0x40) {
		  // AES-128 encryption
		  // Set up AES_ctx struct and input ciphertext.
		  struct AES_ctx ctx;
		  uint8_t in[16];
		  for (int i = 0; i < 16; i += 1) {
			  in[i] = data_buffer[i + 1];
		  }
		  // Initialize AES
		  AES_init_ctx(&ctx, aes_keys);

		  // Decrypt AES
		  AES_ECB_decrypt(&ctx, in);

		  // Copy output of decryption into data buffer.
		  for (int i = 1; i < 17; i += 1) {
			  data_buffer[i] = in[i - 1];
		  }
	  } else {
		  unknownProtocol = true;
	  }

	  // 2. Sends data via the UART peripheral (format: 8N1)
	  // Sends data formatted as follows:
	  // <Success: 1 byte><Data: 16 bytes> - MSByte -> LSByte

	  // If unknown protocol byte was received.
	  if (unknownProtocol) {
		  data_buffer[0] = 0x20;
	  } else {
		  // If decryption was successful:
		  if (data_buffer[1] == 0xFE) {
			  data_buffer[0] = 0x00;
		  }
		  // If decryption was unsuccessful:
		  else {
			  data_buffer[0] = 0x01;
		  }
	  }

	  // Sends data back out.
	  HAL_UART_Transmit(&huart1, data_buffer, 17, HAL_MAX_DELAY);


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : VCP_TX_Pin VCP_RX_Pin */
  GPIO_InitStruct.Pin = VCP_TX_Pin|VCP_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
