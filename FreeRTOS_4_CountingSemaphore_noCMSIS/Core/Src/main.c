/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Very imp note:
  * 	- Check above whether #include "cmsis_os.h" is commented out
  * 	- remove by default created task related functions
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

// osThreadId normalTaskHandle; -> No Need
/* USER CODE BEGIN PV */

TaskHandle_t HPTHandler; // High Priority task Handler
TaskHandle_t MPTHandler; // Medium Priority task Handler
TaskHandle_t LPTHandler; // Low Priority task Handler
TaskHandle_t VLPTHandler; // Very-Low Priority task Handler

// for semaphore
SemaphoreHandle_t countingSem;

// for resource
int resource[3] = {111, 222, 333};
int indx = 0;

// for uart
uint8_t rx_data = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
// void start_normalTask(void const * argument); -> No Need

/* USER CODE BEGIN PFP */
void HPT_Task(void *pvParameters); // High Priority task Handler function
void MPT_Task(void *pvParameters); // Medium Priority task Handler function
void LPT_Task(void *pvParameters); // Low Priority task Handler function
void VLPT_Task(void *pvParameters); // Very-Low Priority task Handler function
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

  //
  HAL_UART_Receive_IT(&huart3, &rx_data, 1);

  countingSem = xSemaphoreCreateCounting(3, 0);
  if (countingSem == NULL) HAL_UART_Transmit(&huart3, (uint8_t *)"Unable to Create Semaphore\n\r)", 28, 100);
  else HAL_UART_Transmit(&huart3, (uint8_t *)"Counting-Semaphore created successfully\n\r)", 41, 1000);

  // create Tasks
  xTaskCreate(HPT_Task, "HTP", 128, NULL, 3, &HPTHandler);
  xTaskCreate(MPT_Task, "MPT", 128, NULL, 2, &MPTHandler);
  xTaskCreate(LPT_Task, "LPT", 128, NULL, 1, &LPTHandler);
  xTaskCreate(VLPT_Task, "VLPT", 128, NULL, 0, &VLPTHandler);

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
  /*osThreadDef(normalTask, start_normalTask, osPriorityNormal, 0, 128); -> No Need
  normalTaskHandle = osThreadCreate(osThread(normalTask), NULL);*/

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  // osKernelStart(); -> No Need

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
// High Priority task Handler function
void HPT_Task(void *pvParameters)
{
	char sresource[3];
	int cou_sem = 0;
	char s_cou_sem[2];

	// Give 3 Semaphores at the beginning...
	xSemaphoreGive(countingSem);
	xSemaphoreGive(countingSem);
	xSemaphoreGive(countingSem);

	while(1)
	{
		char str[150];
		strcpy(str, "Entered HPT Task.\nAbout to Acquire the Semaphore\n\r");
		cou_sem = uxSemaphoreGetCount(countingSem);
		itoa(cou_sem, s_cou_sem, 10);
		strcat(str, "Token available are: ");
		strcat(str, s_cou_sem);
		strcat(str, "\n");
		HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);

		xSemaphoreTake(countingSem, portMAX_DELAY); // portMAX_DELAY -> wait forever

		// below statements will be executed when Semaphore will be acquired
		itoa(resource[indx], sresource, 10); // itoa -> convert integer into string
		strcpy(str, "Leaving HPT Task.\nAccessed Data: ");
		strcat(str, sresource);
		strcat(str, "\nNot releasing the Semaphore\n\r");
		HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);

		indx++;
		if(indx>2) indx=0;

		vTaskDelay(3000); // delay of 3 sec
//		vTaskDelete(NULL);
	}
}

// Medium Priority task Handler function
void MPT_Task(void *pvParameters)
{
	char sresource[3];
	int cou_sem = 0;
	char s_cou_sem[2];

	while(1)
	{
		char str[150];
		strcpy(str, "Entered MPT Task.\nAbout to Acquire the Semaphore\n\r");
		cou_sem = uxSemaphoreGetCount(countingSem);
		itoa(cou_sem, s_cou_sem, 10);
		strcat(str, "Token available are: ");
		strcat(str, s_cou_sem);
		strcat(str, "\n");
		HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);

		xSemaphoreTake(countingSem, portMAX_DELAY); // portMAX_DELAY -> wait forever

		// below statements will be executed when Semaphore will be acquired
		itoa(resource[indx], sresource, 10); // itoa -> convert integer into string
		strcpy(str, "Leaving MPT Task.\nAccessed Data: ");
		strcat(str, sresource);
		strcat(str, "\nNot releasing the Semaphore\n\r");
		HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);

		indx++;
		if(indx>2) indx=0;

		vTaskDelay(2000); // delay of 3 sec
//		vTaskDelete(NULL);
	}
}

// Low Priority task Handler function
void LPT_Task(void *pvParameters)
{
	char sresource[3];
	int cou_sem = 0;
	char s_cou_sem[2];

	while(1)
	{
		char str[150];
		strcpy(str, "Entered LPT Task.\nAbout to Acquire the Semaphore\n\r");
		cou_sem = uxSemaphoreGetCount(countingSem);
		itoa(cou_sem, s_cou_sem, 10);
		strcat(str, "Token available are: ");
		strcat(str, s_cou_sem);
		strcat(str, "\n");
		HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);

		xSemaphoreTake(countingSem, portMAX_DELAY); // portMAX_DELAY -> wait forever

		// below statements will be executed when Semaphore will be acquired
		itoa(resource[indx], sresource, 10); // itoa -> convert integer into string
		strcpy(str, "Leaving LPT Task.\nAccessed Data: ");
		strcat(str, sresource);
		strcat(str, "\nNot releasing the Semaphore\n\r");
		HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);

		indx++;
		if(indx>2) indx=0;

		vTaskDelay(1000); // delay of 3 sec
//		vTaskDelete(NULL);
	}
}
// Very-Low Priority task Handler function
void VLPT_Task(void *pvParameters)
{
	char sresource[3];
	int cou_sem = 0;
	char s_cou_sem[2];

	while(1)
	{
		char str[150];
		strcpy(str, "Entered VLPT Task.\nAbout to Acquire the Semaphore\n\r");
		cou_sem = uxSemaphoreGetCount(countingSem);
		itoa(cou_sem, s_cou_sem, 10);
		strcat(str, "Token available are: ");
		strcat(str, s_cou_sem);
		strcat(str, "\n");
		HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);

		xSemaphoreTake(countingSem, portMAX_DELAY); // portMAX_DELAY -> wait forever

		// below statements will be executed when Semaphore will be acquired
		itoa(resource[indx], sresource, 10); // itoa -> convert integer into string
		strcpy(str, "Leaving VLPT Task.\nAccessed Data: ");
		strcat(str, sresource);
		strcat(str, "\nNot releasing the Semaphore\n\r");
		HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);

		indx++;
		if(indx>2) indx=0;

		vTaskDelay(3000); // delay of 3 sec
	}
}

// UART Call-back Function
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(huart, &rx_data, 1);
	if (rx_data == 'r')
	{
		// release the semaphore
		/*
		 * the xHigherPriorityTaskWoken parameter must be initialized to pdFALSE as
		 * it will get set to pdTRUE inside the interrupt safe API function if a context
		 * switch is required.
		 */
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		// releasing all 3 tokens - free tokens as per needed on receiving r
		/*
		 * Counting Semaphore can be released from anywhere in the code as per needed
		 */
		xSemaphoreGiveFromISR(countingSem, &xHigherPriorityTaskWoken); // ISR Safe Version
		xSemaphoreGiveFromISR(countingSem, &xHigherPriorityTaskWoken); // ISR Safe Version
		xSemaphoreGiveFromISR(countingSem, &xHigherPriorityTaskWoken); // ISR Safe Version

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
/*void start_normalTask(void const * argument) -> No Need
{
  // USER CODE BEGIN 5
  // Infinite loop
  for(;;)
  {
    osDelay(1);
  }
  // USER CODE END 5
}*/

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
