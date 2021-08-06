/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "mlx90632.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CHIP_ADDRESS 0x3a << 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* HAL_I2C_Mem_Read()/Write() are used instead of Master_Transmit()/Receive() because repeated start condition is needed */
/* Implementation of I2C read for 16-bit values */
int32_t mlx90632_i2c_read(int16_t register_address, uint16_t *value)
{
	uint8_t data[2];
	int32_t ret;
	ret = HAL_I2C_Mem_Read(&hi2c1, CHIP_ADDRESS, register_address, 2, data, sizeof(data), 100);
	//Endianness
	*value = data[1]|(data[0]<<8);
	return ret;
}

/* Implementation of I2C read for 32-bit values */
int32_t mlx90632_i2c_read32(int16_t register_address, uint32_t *value)
{
	uint8_t data[4];
	int32_t ret;
	ret = HAL_I2C_Mem_Read(&hi2c1, CHIP_ADDRESS, register_address, 2, data, sizeof(data), 100);
	//Endianness
	*value = data[2]<<24|data[3]<<16|data[0]<<8|data[1];
	return ret;
}

/* Implementation of I2C write for 16-bit values */
int32_t mlx90632_i2c_write(int16_t register_address, uint16_t value) {
	uint8_t data[2];
	data[0] = value >> 8;
	data[1] = value;
	return HAL_I2C_Mem_Write(&hi2c1, CHIP_ADDRESS, register_address, 2, data, 2, 100);
}

/* Implementation of reading all calibration parameters for calucation of Ta and To */
static int mlx90632_read_eeprom(int32_t *PR, int32_t *PG, int32_t *PO, int32_t *PT, int32_t *Ea, int32_t *Eb, int32_t *Fa, int32_t *Fb, int32_t *Ga, int16_t *Gb, int16_t *Ha, int16_t *Hb, int16_t *Ka)
{
	int32_t ret;
	ret = mlx90632_i2c_read32(MLX90632_EE_P_R, (uint32_t *) PR);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read32(MLX90632_EE_P_G, (uint32_t *) PG);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read32(MLX90632_EE_P_O, (uint32_t *) PO);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read32(MLX90632_EE_P_T, (uint32_t *) PT);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read32(MLX90632_EE_Ea, (uint32_t *) Ea);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read32(MLX90632_EE_Eb, (uint32_t *) Eb);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read32(MLX90632_EE_Fa, (uint32_t *) Fa);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read32(MLX90632_EE_Fb, (uint32_t *) Fb);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read32(MLX90632_EE_Ga, (uint32_t *) Ga);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read(MLX90632_EE_Gb, (uint16_t *) Gb);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read(MLX90632_EE_Ha, (uint16_t *) Ha);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read(MLX90632_EE_Hb, (uint16_t *) Hb);
	if(ret < 0)
		return ret;
	ret = mlx90632_i2c_read(MLX90632_EE_Ka, (uint16_t *) Ka);
	if(ret < 0)
		return ret;
	return 0;
}

void usleep(int min_range, int max_range) {
	while(--min_range);
}

double pre_ambient, pre_object, ambient, object;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
mlx90632_init();
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	/* Definition of MLX90632 calibration parameters */
	int16_t ambient_new_raw;
	int16_t ambient_old_raw;
	int16_t object_new_raw;
	int16_t object_old_raw;
	int32_t PR = 0x00587f5b;
	int32_t PG = 0x04a10289;
	int32_t PT = 0xfff966f8;
	int32_t PO = 0x00001e0f;
	int32_t Ea = 4859535;
	int32_t Eb = 5686508;
	int32_t Fa = 53855361;
	int32_t Fb = 42874149;
	int32_t Ga = -14556410;
	int16_t Ha = 16384;
	int16_t Hb = 0;
	int16_t Gb = 9728;
	int16_t Ka = 10752;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	/* Read EEPROM calibration parameters */
	mlx90632_read_eeprom(&PR, &PG, &PO, &PT, &Ea, &Eb, &Fa, &Fb, &Ga, &Gb, &Ha, &Hb, &Ka);

  while (1)
  {
		/* Get raw data from MLX90632 */
		mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw);
		/* Pre-calculations for ambient and object temperature calculation */
		pre_ambient = mlx90632_preprocess_temp_ambient(ambient_new_raw, ambient_old_raw, Gb);
		pre_object = mlx90632_preprocess_temp_object(object_new_raw, object_old_raw, ambient_new_raw, ambient_old_raw, Ka);
		/* Set emissivity = 1 */
		mlx90632_set_emissivity(1.0);
		/* Calculate ambient and object temperature */
		ambient = mlx90632_calc_temp_ambient(ambient_new_raw, ambient_old_raw, PT, PR, PG, PO, Gb);
		object = mlx90632_calc_temp_object(pre_object, pre_ambient, Ea, Eb, Ga, Fa, Fb, Ha, Hb);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
//	  HAL_Delay(10);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 116;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
