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
#include "stdio.h"
#include "i2c_lcd.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t RX_FSRH1S_INDX;
uint8_t RX_HC05_INDX;
uint8_t RX_DATA_FSRH1S[5];
uint8_t RX_DATA_HC05[5];
//uint8_t transfer_cplt;
uint8_t FSRH1S_MAX_LENGTH = 14;
uint8_t HC05_MAX_LENGTH = 4;
uint8_t RX_FSRH1S_BUFFER[14];
uint8_t RX_HC05_BUFFER[4];
uint8_t flag= 0;
I2C_HandleTypeDef hi2c1;
i2cLcd_HandleTypeDef h_lcd;
char strData[32];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void RFID_SECERN_USER();
void HC05_identification();
void i2cLcd_HelloGuest();
void i2cLcd_intruder();
void i2cLcd_HelloUser();
void PWM_Serovomotor();
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t i2c_lcd_addr = (0x27<<1);
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
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, RX_DATA_FSRH1S, 1);
  HAL_UART_Receive_IT(&huart3, RX_DATA_HC05, 1); // sss
  HAL_UART_Receive_IT(&huart1, RX_DATA_FSRH1S, 1);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2); // Add PWM Servo Motor
  i2cLcd_CreateHandle(&h_lcd, &hi2c1, i2c_lcd_addr);
  i2cLcd_Init(&h_lcd);
  //i2cLcd_ClearDisplay(&h_lcd);
  //sprintf(strData,"Hello world!!");
  // HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  while (1)
	  {
		  switch(flag)
		  {
		  	  case 0 :
		  		  i2cLcd_HelloGuest();
			  	  break;
		  	  case 1 :
		  		  i2cLcd_HelloUser();
		  		  PWM_Serovomotor();
		  		  break;
		  	  case 2 :
		  		  i2cLcd_intruder();
		  		  break;
		  	  default :
		  		  break;
		  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.OwnAddress1 = 254;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1440-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 38400;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
 // HAL_UART_Transmit(&huart2, RX_DATA_FSRH1S, 6,100);
  uint8_t i;
  uint8_t j;
  /* UART1 = RFID UART2 = main computer UART3 = HC-05*/
  if(huart ->Instance == USART1)// && huart -> Instance != USART3
  {
	  	  if(RX_FSRH1S_INDX == 0 || RX_FSRH1S_INDX == FSRH1S_MAX_LENGTH) //rx_index Arry Initialization
	  	  {
	  		  for(i=0;i<FSRH1S_MAX_LENGTH;i++)
	  		  {
	  			  RX_FSRH1S_BUFFER[i] = 0;
	  		  }
	  		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);

	  		  if(RX_FSRH1S_INDX == FSRH1S_MAX_LENGTH)
	  		  {
	  			  RX_FSRH1S_INDX = 0;
	  		  }
	  	  }

	  	  if(RX_DATA_FSRH1S[0] != 13)
	  	  {
	  		RX_FSRH1S_BUFFER[RX_FSRH1S_INDX++] = RX_DATA_FSRH1S[0];
	  	  }

	  	  RFID_SECERN_USER();
	  HAL_UART_Receive_IT(&huart1, RX_DATA_FSRH1S, 1); //RFID Tag receive!!
  }
  if(huart ->Instance == USART3)
  {
		  if(RX_HC05_INDX == 0 || RX_HC05_INDX == HC05_MAX_LENGTH) //rx_index Arry Initialization
		  {
			  for(j=0;j<HC05_MAX_LENGTH;j++)
			  {
				  RX_HC05_BUFFER[j] = 0;
			  }
				  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);

			  if(RX_HC05_INDX == HC05_MAX_LENGTH)
			  {
				  RX_HC05_INDX = 0;
			  }
		  }
		  if(RX_DATA_HC05[0] != 13)
		  {
			  RX_HC05_BUFFER[RX_HC05_INDX++] = RX_DATA_HC05[0];
		  }
		  HC05_identification();
  }

  	  HAL_UART_Receive_IT(&huart3, RX_DATA_HC05, 1);

}

void RFID_SECERN_USER() //get UID code and secern
{
			if(RX_FSRH1S_BUFFER[0] == 0x33 && RX_FSRH1S_BUFFER[1] == 0x0E && RX_FSRH1S_BUFFER[2] == 0x1A)
		  {
		  		  if(RX_FSRH1S_BUFFER[5] == 0x6D && RX_FSRH1S_BUFFER[6] == 0x16 && RX_FSRH1S_BUFFER[7] == 0x21 && RX_FSRH1S_BUFFER[8] == 0xC2 && RX_FSRH1S_BUFFER[9] == 0x50 && RX_FSRH1S_BUFFER[10] == 0x01 && RX_FSRH1S_BUFFER[11] == 0x04 && RX_FSRH1S_BUFFER[12] == 0xE0 && RX_FSRH1S_BUFFER[13] == 0x99)
				{
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
					flag =1;
				}
		  		  else if(RX_FSRH1S_BUFFER[5] == 0x2D && RX_FSRH1S_BUFFER[6] == 0x00 && RX_FSRH1S_BUFFER[7] == 0x21 && RX_FSRH1S_BUFFER[8] == 0xC2 && RX_FSRH1S_BUFFER[9] == 0x50 && RX_FSRH1S_BUFFER[10] == 0x01 && RX_FSRH1S_BUFFER[11] == 0x04 && RX_FSRH1S_BUFFER[12] == 0xE0 && RX_FSRH1S_BUFFER[13] == 0x99)
		  		{
		  			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
		  			flag =1;
		  		}
		  		  else if(RX_FSRH1S_BUFFER[5] == 0x42 && RX_FSRH1S_BUFFER[6] == 0x1D && RX_FSRH1S_BUFFER[7] == 0x21 && RX_FSRH1S_BUFFER[8] == 0xC2 && RX_FSRH1S_BUFFER[9] == 0x50 && RX_FSRH1S_BUFFER[10] == 0x01 && RX_FSRH1S_BUFFER[11] == 0x04 && RX_FSRH1S_BUFFER[12] == 0xE0 && RX_FSRH1S_BUFFER[13] == 0x99)
		  		{
		  			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
		  			flag =1;
		  		}
		  		else
		  			flag = 2;
		  }
}
void HC05_identification()
{

	if(RX_HC05_BUFFER[0] == 0x31 && RX_HC05_BUFFER[1] == 0x32 && RX_HC05_BUFFER[2] == 0x33 && RX_HC05_BUFFER[3] == 0x34)
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
		flag = 1;
	}
	else
		flag = 2;
}

void i2cLcd_HelloGuest()
{
	      uint8_t i=0;

		  i2cLcd_ClearDisplay(&h_lcd);
		  HAL_Delay(20);
		  i2cLcd_SetCursorPosition(&h_lcd, 0x00);
		  sprintf(strData,"Hello guest!!");
		  HAL_Delay(20);
			  while(strData[i])
			  {
				  i2cLcd_SendChar(&h_lcd, strData[i]);
				  HAL_Delay(100);
				  i++;
			  }
		  i2cLcd_SetCursorPosition(&h_lcd, 0x40);
		  i=0;
		  sprintf(strData,"Tag your UID");
			  while(strData[i])
			  {
				  i2cLcd_SendChar(&h_lcd, strData[i]);
				  HAL_Delay(100);
				  i++;
			  }
}

void i2cLcd_intruder()
{
	        uint8_t i=0;
			i2cLcd_ClearDisplay(&h_lcd);
			HAL_Delay(20);
			i2cLcd_SetCursorPosition(&h_lcd, 0x00);
			sprintf(strData,"Error!! Error!");
			HAL_Delay(20);
			while(strData[i])
			{
				i2cLcd_SendChar(&h_lcd, strData[i]);
				HAL_Delay(100);
				i++;
			}

}

void i2cLcd_HelloUser()
{
	        uint8_t i=0;
			i2cLcd_ClearDisplay(&h_lcd);
			HAL_Delay(20);
			i2cLcd_SetCursorPosition(&h_lcd, 0x00);
			sprintf(strData,"Welcome!!");
			HAL_Delay(20);
			while(strData[i])
			{
				i2cLcd_SendChar(&h_lcd, strData[i]);
				HAL_Delay(100);
				i++;
			}
			i2cLcd_SetCursorPosition(&h_lcd, 0x40);
			i=0;
			sprintf(strData,"open the door!");
			 while(strData[i])
		    {
				i2cLcd_SendChar(&h_lcd, strData[i]);
				HAL_Delay(100);
				i++;
		    }

}

void PWM_Serovomotor()
{
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2,125); //open
		HAL_Delay(3000); // 1000ms
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 25); //close
		HAL_Delay(500); // 1000ms
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  }
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
  /* User can add his own implementation to report the file name and line FSRH1S_MAX_LENGTH,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
