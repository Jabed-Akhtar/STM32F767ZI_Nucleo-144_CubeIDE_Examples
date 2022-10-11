/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Very imp note:
  * 	- comment out #include "cmsis_os.h" , as it is not used
  * 	- comment or remote by-default created task related stuffs
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include <stdio.h>
#include "stdlib.h"
#include <string.h>

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

UART_HandleTypeDef huart3;

//osThreadId normalTaskHandle;
/* USER CODE BEGIN PV */

// defining Task Handlers
xTaskHandle Sender_HPT_Handler;
xTaskHandle Sender_LPT_Handler;
xTaskHandle Receiver_Handler;

// defining Queue Handlers
xQueueHandle simpleQueue;

uint8_t Rx_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
//void start_normalTask(void const * argument);

/* USER CODE BEGIN PFP */

void Sender_HPT_Task(void *argument); // Sender High Priority task Handler function
void Sender_LPT_Task(void *argument); // Sender Low Priority task Handler function
void Receiver_Task(void *argument); // Receiver task Handler function

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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // sending data through USART at the beginning of Programm
  char stri[] = "\n---------- Programm started!!! ----------\n\r";
  HAL_UART_Transmit(&huart3, (uint8_t*)stri, strlen(stri), HAL_MAX_DELAY); //HAL_MAX_DELAY -> 0xFFFFFFFFU

  // creating Queue
  simpleQueue = xQueueCreate(5, sizeof(int));
  if (simpleQueue == 0) // if Queue not created
  {
	  char *str1 = "Unable to create Queue.\n\r";
	  HAL_UART_Transmit(&huart3, (uint8_t*)str1, strlen(str1), HAL_MAX_DELAY);
  }
  else
  {
	  char *str1 = "Integer Queue successfully created.\n\r";
	  HAL_UART_Transmit(&huart3, (uint8_t*)str1, strlen(str1), HAL_MAX_DELAY);
  }

  // create Tasks
  xTaskCreate(Sender_HPT_Task, "Sender_HTP", 128, NULL, 3, &Sender_HPT_Handler);
  xTaskCreate(Sender_LPT_Task, "Sender_LTP", 128, (void *)111, 2, &Sender_LPT_Handler);
  xTaskCreate(Receiver_Task, "Receive", 128, NULL, 1, &Receiver_Handler);

  HAL_UART_Receive_IT(&huart3, &Rx_data, 1);

  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of normalTask */
//  osThreadDef(normalTask, start_normalTask, osPriorityNormal, 0, 128);
//  normalTaskHandle = osThreadCreate(osThread(normalTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
//  osKernelStart();

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// Sender High Priority task Handler function
void Sender_HPT_Task(void *argument)
{
	int i=222;
	uint32_t tickDelay = pdMS_TO_TICKS(2000);
	while(1)
	{
		char *str1 = "Entered Sender_HPT_Task.\n\rAbout to Send a number To the Queue.\n\r";
		HAL_UART_Transmit(&huart3, (uint8_t*)str1, strlen(str1), HAL_MAX_DELAY);

		if(xQueueSend(simpleQueue, &i, portMAX_DELAY) == pdPASS)
		{
			char *str2 = "Successfully sent number to the Queue.\n\rLeaving Sender_HPT_Task.\n\r";
			HAL_UART_Transmit(&huart3, (uint8_t*)str2, strlen(str2), HAL_MAX_DELAY);
		}

		vTaskDelay(tickDelay);
	}
}

// Sender Low Priority task Handler function
void Sender_LPT_Task(void *argument)
{
	int toSend;
	uint32_t tickDelay = pdMS_TO_TICKS(1000);
	while(1)
	{
		toSend = (int) argument;
		char *str1 = "Entered Sender_LPT_Task.\n\rAbout to Send a number To the Queue.\n\r";
		HAL_UART_Transmit(&huart3, (uint8_t*)str1, strlen(str1), HAL_MAX_DELAY);

		xQueueSend(simpleQueue, &toSend, portMAX_DELAY);

		char *str2 = "Successfully sent number to the Queue.\n\rLeaving Sender_LPT_Task.\n\r";
		HAL_UART_Transmit(&huart3, (uint8_t*)str2, strlen(str2), HAL_MAX_DELAY);

		vTaskDelay(tickDelay);
	}
}

// Receiver task Handler function
void Receiver_Task(void *argument)
{
	int received = 0;
	uint32_t tickDelay = pdMS_TO_TICKS(5000);
	while(1)
	{
		char str[100];
		strcpy(str, "Entered Receiver_Task.\n\rAbout to Receive a number From the Queue.\n\r");
		HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);

		if(xQueueReceive(simpleQueue, &received, portMAX_DELAY) != pdTRUE)
		{
			HAL_UART_Transmit(&huart3, (uint8_t*)"Error in Receiving from Queue.\n\r", 32, 1000);
		}
		else
		{
			sprintf(str, "Successfully Received Number %d from the Queue.\n\rLeaving Receiver_Task.\n\r", received);
			HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
		}

		vTaskDelay(tickDelay);
	}
}

// Callback function for UART_Rx_Interrupt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(huart, &Rx_data, 1);
	int toSend = 1234567890;
	char str[100];
	if (Rx_data == 'r')
	{
		/*
		 * the xHigherPriorityTaskWoken parameter must be initialized to pdFALSE as
		 * it will get set to pdTRUE inside the interrupt safe API function if a context
		 * switch is required.
		 */
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		if(xQueueSendToFrontFromISR(simpleQueue, &toSend, &xHigherPriorityTaskWoken) == pdPASS)
		{
			sprintf(str, "Number %d Sent from ISR to Queue.\n", toSend);
			HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
		}

		/*
		 * Pass the xHigherPriorityTaskWoken value into portEND_SWITCHING_ISR(). If
		 * xHigherPriorityTaskWoken was set to pdTRUE inside xSemaphoreGiveFromISR()
		 * xHigherPriorityTaskWoken is still pdFALSE then calling portEND_SWITCHING_ISR()
		 * will have no effect.
		 */
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken); // {}-> needed to write this in every ISR function
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_start_normalTask */
/**
  * @brief  Function implementing the normalTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_start_normalTask */
//void start_normalTask(void const * argument)
//{
//  /* USER CODE BEGIN 5 */
//  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
//  /* USER CODE END 5 */
//}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
