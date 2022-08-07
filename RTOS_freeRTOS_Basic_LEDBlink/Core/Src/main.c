/********************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @author         : Jabed-Akhtar
  * @date           : 13.03.2022
  ******************************************************************************
  * Description:
  *		- LED: PB7(Blue) and PB14(Red) (for testing purpose)
  *		- freeRTOS:
  *			|- thread1: blinkLEDBlueHandle -> thread to blink BlueLED every 500ms
  *				-> task is programmed inside StartBlinkLEDBlue(void)
  *			|- thread2: blinkLEDRed -> thread to blink RedLED every 600ms
  *				-> task is programmed inside StartBlinkLEDRed(void)
  *******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"


/* Threads definitions -------------------------------------------------------*/
/* Definitions for blinkLEDBlue */
osThreadId_t blinkLEDBlueHandle;
const osThreadAttr_t blinkLEDBlue_attributes = {
  .name = "blinkLEDBlue",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for blinkLEDRed */
osThreadId_t blinkLEDRedHandle;
const osThreadAttr_t blinkLEDRed_attributes = {
  .name = "blinkLEDRed",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartBlinkLEDBlue(void *argument);
void StartBlinkLEDRed(void *argument);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* Init scheduler */
  osKernelInitialize();

  /* Create the thread(s) ---------------*/
  /* creation of blinkLEDBlue */
  blinkLEDBlueHandle = osThreadNew(StartBlinkLEDBlue, NULL, &blinkLEDBlue_attributes);
  /* creation of blinkLEDRed */
  blinkLEDRedHandle = osThreadNew(StartBlinkLEDRed, NULL, &blinkLEDRed_attributes);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  while (1)
  {
    // Leaving this empty - task is taken by freeRTOS scheduler
  }
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB14 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN Header_StartBlinkLEDBlue */
/**
  * @brief  Function implementing the blinkLEDBlue thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBlinkLEDBlue */
void StartBlinkLEDBlue(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	GPIOB->BSRR |= (1<<7); // set pin PB7 in register GPIOx_BSRR
    osDelay(500);
    GPIOB->BSRR |= ((1<<7)<<16); // reset pin PB7 in register GPIOx_BSRR
    osDelay(500);
  }
  // In case the sw accidently exit from task loop
  osThreadTerminate(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartBlinkLEDRed */
/**
* @brief Function implementing the blinkLEDRed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlinkLEDRed */
void StartBlinkLEDRed(void *argument)
{
  /* USER CODE BEGIN StartBlinkLEDRed */
  /* Infinite loop */
  for(;;)
  {
	GPIOB->BSRR |= (1<<14); // set pin PB14 in register GPIOx_BSRR
    osDelay(600);
    GPIOB->BSRR |= ((1<<14)<<16); // reset pin PB14 in register GPIOx_BSRR
    osDelay(600);
    osThreadTerminate(NULL);
  }
  /* USER CODE END StartBlinkLEDRed */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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

