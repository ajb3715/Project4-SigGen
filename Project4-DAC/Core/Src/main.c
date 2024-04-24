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
#include "Signal_Generator.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;
DMA_HandleTypeDef hdma_dac_ch2;

RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

struct user_command *command;

/* Definitions for ProcessCommand */
osThreadId_t ProcessCommandHandle;
const osThreadAttr_t ProcessCommand_attributes = {
  .name = "ProcessCommand",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RecieveCommand */
osThreadId_t RecieveCommandHandle;
const osThreadAttr_t RecieveCommand_attributes = {
  .name = "RecieveCommand",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CommandQueue */
osMessageQueueId_t CommandQueueHandle;
const osMessageQueueAttr_t CommandQueue_attributes = {
  .name = "CommandQueue"
};
/* Definitions for MUTEX */
osMutexId_t MUTEXHandle;
const osMutexAttr_t MUTEX_attributes = {
  .name = "MUTEX"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_RNG_Init(void);
void StartProcessCommand(void *argument);
void StartRecieveCommand(void *argument);

/* USER CODE BEGIN PFP */

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
	command = (struct user_command *)malloc(sizeof(struct user_command));
		if(command == NULL){
			exit(99);
		}
		command->channel = 0;
		command->frequency = 0.0;
		command->maxv = 0.0;
		command->minv = 0.0;
		command->noise = 0;
		command->wave = 'n';

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim5);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of MUTEX */
  MUTEXHandle = osMutexNew(&MUTEX_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CommandQueue */
  CommandQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &CommandQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ProcessCommand */
  ProcessCommandHandle = osThreadNew(StartProcessCommand, NULL, &ProcessCommand_attributes);

  /* creation of RecieveCommand */
  RecieveCommandHandle = osThreadNew(StartRecieveCommand, NULL, &RecieveCommand_attributes);

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
  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T5_TRGO;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

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
  htim2.Init.Period = 1000;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartProcessCommand */
/**
  * @brief  Function implementing the ProcessCommand thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartProcessCommand */
void StartProcessCommand(void *argument)
{
  /* USER CODE BEGIN 5 */
	char command_buffer[100];
	char print_buffer[256];
	int i = 0;
	int print_size = 0;
	int valid_entry = 0;
  /* Infinite loop */
  for(;;)
  {
	    osMutexAcquire(MUTEXHandle, osWaitForever);
	  	uint8_t c = 0;
	  	HAL_UART_Receive(&huart2, &c, 1, 100);					// Read and print inputted char
	  	HAL_UART_Transmit(&huart2, &c, 1, 100);

	  	if ((char)c == '\r'){
//	  		if enter is pressed, process command to see if valid
	  		command_buffer[i] = '\r';
	  		command_buffer[i+1] = '\n';
	  		command_buffer[i+2] = '\0';
	  		print_size = sprintf(print_buffer, command_buffer);
	  		HAL_UART_Transmit(&huart2, (uint8_t*)print_buffer, print_size, 100);		// prints inputted command
	  		char* word = strtok(command_buffer, " ");									// split string to just command name
	  		i = 0;																		// reset index
	  		valid_entry = 1;															// by default, valid input - later conditions alter if needed

	  		word = strtok(NULL, " ");													// split to next info
	  		int ivalue = atoi(word);
	  		if (ivalue >= 3 || ivalue <= 0){											// check if channel value valid
	  			valid_entry = 0;														// if not make command invalid
	  			print_size = sprintf(print_buffer, "Channel value must be 1 or 2\r\n");
	  			HAL_UART_Transmit(&huart2, (uint8_t*)print_buffer, print_size, 100);
	  		}
	  		command->channel = ivalue;

	  		word = strtok(NULL, " ");
	  		if (*word != 'A' && *word != 'R' && *word != 'S' && *word != 'T'){			// check if wave type is valid
	  			valid_entry = 0;														// if not make command invalid
	  			print_size = sprintf(print_buffer, "Wave type must be S = sine, T = triangle, R = rectangle or A = arbitrary/EKG\r\n");
	  			HAL_UART_Transmit(&huart2, (uint8_t*)print_buffer, print_size, 100);
	  		}
	  		command->wave = *word;

	  		word = strtok(NULL, " ");
	  		double fvalue = atof(word);
	  		if ((fvalue > 10000 || fvalue < 0.5) && fvalue != 0){							// check if frequncy value is valid
	  			valid_entry = 0;														// if not make command invalid
	  			print_size = sprintf(print_buffer, "Frequency must be between 0.5 Hz and 10 kHz, or 0 for DC\r\n");
	  			HAL_UART_Transmit(&huart2, (uint8_t*)print_buffer, print_size, 100);
	  		}
	  		command->frequency = fvalue;

	  		word = strtok(NULL, " ");
	  		fvalue = atof(word);
	  		if (fvalue > 3.3 || fvalue < 0){											// check if min voltage value is valid
	  			valid_entry = 0;														// if not make command invalid
	  			print_size = sprintf(print_buffer, "Min Voltage must be between 0v and 3.3v\r\n");
	  			HAL_UART_Transmit(&huart2, (uint8_t*)print_buffer, print_size, 100);
	  		}
	  		command->minv = fvalue;
	  		print_size = sprintf(print_buffer, "Min Voltage %f, %f\r\n", command->minv, fvalue);
	  		HAL_UART_Transmit(&huart2, (uint8_t*)print_buffer, print_size, 100);

	  		word = strtok(NULL, " ");
	  		fvalue = atof(word);
	  		if (fvalue > 3.3 || fvalue < 0){											// check if max voltage value is valid
	  			valid_entry = 0;														// if not make command invalid
	  			print_size = sprintf(print_buffer, "Max Voltage must be between 0v and 3.3v\r\n");
	  			HAL_UART_Transmit(&huart2, (uint8_t*)print_buffer, print_size, 100);
	  		} else if (ivalue <= command->minv){										// check if max voltage value is less than min voltage
	  			valid_entry = 0;														// if not make command invalid
	  			print_size = sprintf(print_buffer, "Max Voltage must be between less than Min Voltage\r\n");
	  			HAL_UART_Transmit(&huart2, (uint8_t*)print_buffer, print_size, 100);
	  		}
	  		command->maxv = fvalue;

	  		word = strtok(NULL, " ");
	  		ivalue = atoi(word);
	  		if (ivalue > 12 || ivalue < 0){												// check if noise value is valid
	  			valid_entry = 0;														// if not make command invalid
	  			print_size = sprintf(print_buffer, "Noise value must be between 0 and 12 (inclusive)\r\n");
	  			HAL_UART_Transmit(&huart2, (uint8_t*)print_buffer, print_size, 100);
	  		}
	  		command->noise = ivalue;

  			if (valid_entry){													// if command is valid, then add to queue
				osMessageQueuePut(CommandQueueHandle, &command, 0, 0);
				valid_entry = 0;
  			}

	  	} else if (c != 0){						// if character is valid and not enter key
	  		command_buffer[i] = c;				// add to buffer to save
	  		i++;
	  	}
	  	osMutexRelease(MUTEXHandle);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartRecieveCommand */
/**
* @brief Function implementing the RecieveCommand thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRecieveCommand */
void StartRecieveCommand(void *argument)
{
  /* USER CODE BEGIN StartRecieveCommand */
  /* Infinite loop */
  for(;;)
  {
  osMutexAcquire(MUTEXHandle, osWaitForever);
			  if(osMessageQueueGetCount(CommandQueueHandle) != 0){
				  char buf[256];
				  struct user_command *cmd = (struct user_command *)malloc(sizeof(struct user_command));
				  if(cmd == NULL){
					  exit(98);
				  }
				  osMessageQueueGet(CommandQueueHandle, &cmd, 0, 0);
				  sig_gen(cmd, &hrng, &hdac1);
				  sprintf(buf,"\r\n Enter another wave generation! \r\n");
				  HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), 100);
				  free(cmd);

		  }

			  osMutexRelease(MUTEXHandle);
		   vTaskDelay(100);
  }
  /* USER CODE END StartRecieveCommand */
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
