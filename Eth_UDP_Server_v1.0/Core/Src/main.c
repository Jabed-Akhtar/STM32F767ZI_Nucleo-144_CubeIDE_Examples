/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  * 	-> Keep this as backup. Don't change/edit/update this project.
  * 	-> Test_2B | fromS
  * 	-> Tag: DONE | WORKING | :)
  *
  *		>> improved:
  *			|- FreeRTOS Debugging added
  * 		|- new queue added to send temperature data also through UART
  *
  * Description: Ethernet Programming with implementation of UDP Server
  * 		   : Tasks:- 1. Heart beat implementation using software timer and visualized by
  * 		   				toggeling green led
  * 		          :- 2. Reading internal temperature sensor at 1 kHz frequency and
  * 		                sending the current and average raw value to the gate keeper.
  * 		                Also visualized by toggeling Blue led
  * 		          :- 3. Ethernet UDP server implementation. Receiving the messages
  * 		                from UDP client and sending it to the gate keeper
  * 		                by implementing ECHO function (Visualized by toggeling Red led).
  * 		          :- 4. Gate keeper implementation. Sending the messages via UDP interface.
  *
  *		-TIM11: Used for Debugging purpose (108MHz/(1079+1) = 100 KHz)
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lwip/sockets.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HEART_BEAT_RATE 250				// Heart beat frequency (2Hz)
#define TEMP_READING_RATE 1000			// Temp. reading frequency (1Hz)
#define OWN_PORT 6799					// available server port
#define OWN_IP_ADDR "192.168.178.31"	// available server ip address
#define CLIENT_PORT 7003				// client port
#define TRUE 1
#define FALSE 0
#define BUFFER_LENGTH (40UL)  // Buffer length
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart3;

//osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
//TimerHandle_t HeartBeat_Timer;
//BaseType_t Timer_State;
QueueHandle_t Own_Queue;
QueueHandle_t temperatureVal_queue;
int socket_id;			// variable for socket id
// Struct variable to pass data to gate keeper task
typedef struct queue_struct{
	in_addr_t addr;
	in_port_t port;
	char msg_buffer[BUFFER_LENGTH];
}queue_struct;
typedef struct tempVal_struct{
	uint32_t lastVal;
	uint32_t avrVal;
}tempVal_struct;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM11_Init(void);
//void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void vTemp_Task(void *pvParameters);			// Temperature task
void HeartBeatCallback(TimerHandle_t xTimer);	// Software timer
void vGateKeeper_Task(void *pvParameters);		// Gaterkeeper task
void vUDP_Task(void *pvParameters);				// UDP task
void vAplicationIdleHook(void);					// Idle Hook task
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
  TimerHandle_t HeartBeat_Timer;
  BaseType_t Timer_State;


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
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  // sending data through USART at the beginning of Programm

  MX_LWIP_Init();

  char stri[] = "\n---------- Programm started!!! ----------\n\n\r";
  HAL_UART_Transmit(&huart3, (uint8_t*)stri, strlen(stri), HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  Own_Queue = xQueueCreate(3,sizeof(struct queue_struct));
  temperatureVal_queue = xQueueCreate(5, sizeof(tempVal_struct));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  HeartBeat_Timer = xTimerCreate("HeartBeat", pdMS_TO_TICKS(HEART_BEAT_RATE), pdTRUE, 0, HeartBeatCallback);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
//  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
//  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(vTemp_Task, "Temp_Task",configMINIMAL_STACK_SIZE*2, NULL,1, NULL);		// Stacksize 128kB
  xTaskCreate(vGateKeeper_Task,"GateKeeper",4*configMINIMAL_STACK_SIZE,NULL,2,NULL);	// Stacksize 4*128kB
  xTaskCreate(vUDP_Task,"UDP_Task",4*configMINIMAL_STACK_SIZE,NULL,1,NULL);					// Stacksize 4*128kB

  if(HeartBeat_Timer != NULL){
	  Timer_State = xTimerStart(HeartBeat_Timer,0);
	  if(Timer_State == pdPASS){
		  vTaskStartScheduler();
	  }
  }


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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1079;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Green_LED_Pin|Red_LED_Pin|Blue_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Green_LED_Pin Red_LED_Pin Blue_LED_Pin */
  GPIO_InitStruct.Pin = Green_LED_Pin|Red_LED_Pin|Blue_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* Idle Hook task */
void vAplicationIdleHook(void){
	__WFI();
}

/* Timer callback function */
void HeartBeatCallback(TimerHandle_t xTimer){
	HAL_GPIO_TogglePin(GPIOB, Green_LED_Pin);
}

/* Temperature reading task */
void vTemp_Task(void *pvParameters)
{
	char stri1[] = "{vTemp_Task} [Info]	Entered vTemp_Task.\n\r";
    HAL_UART_Transmit(&huart3, (uint8_t*)stri1, strlen(stri1), HAL_MAX_DELAY);

	TickType_t xLastWakeTime;
	static uint16_t counter = 0; // Counter variable to count tempr. reading
	static uint32_t temp_sum = 0; // variable to store sum of temp. value
	uint32_t temp = 0;			// variable to store current temp. value
	uint32_t temp_avr = 0;		// variable to store average temp. value
	queue_struct from_temp_task; // structure variable to store data for gate keeper task
	from_temp_task.port = htons(CLIENT_PORT);	// port for client
	from_temp_task.addr = IPADDR_BROADCAST;	// client address
	// variable for data over uart
	tempVal_struct tempStruct;

	xLastWakeTime = xTaskGetTickCount();

	while(1){

		/* Read temp. adc value */
		HAL_ADC_Start(&hadc1);
		while(HAL_ADC_PollForConversion(&hadc1,1) != HAL_OK){}
		temp = HAL_ADC_GetValue(&hadc1);
		temp_sum += temp;
		counter++;
		temp_avr = temp_sum / counter;	// get average value
		HAL_ADC_Stop(&hadc1);

		memset(&from_temp_task.msg_buffer, 0, BUFFER_LENGTH); // clear the character buffer
		/* make a string */
		sprintf(from_temp_task.msg_buffer,"Cur_Temp= %lu , Avr_Temp= %lu\n",(unsigned long)temp,(unsigned long)temp_avr);

		tempStruct.lastVal = temp;
		tempStruct.avrVal = temp_avr;

		HAL_GPIO_TogglePin(GPIOB, Blue_LED_Pin);	// toggel blue led

		xQueueSendToBack(Own_Queue, &from_temp_task, 10);	// send message to gatekeeper task
		xQueueSendToBack(temperatureVal_queue, &tempStruct, 10);

		vTaskDelayUntil(&xLastWakeTime, TEMP_READING_RATE); // wait till timeout
	}
}

/* UDP task */
void vUDP_Task(void *pvParameters)
{
	char stri1[] = "{vUDP_Task} [Info]	Entered vUDP_Task.\n\r";
	HAL_UART_Transmit(&huart3, (uint8_t*)stri1, strlen(stri1), HAL_MAX_DELAY);

	struct sockaddr_in client_addr; // struct variable for udp client
	struct sockaddr_in server_addr; // struct variable for udp server
	queue_struct from_udp_task;		// struct variable to pass through gate keeper
	uint8_t echo_flag = FALSE;		// echo flag
	socklen_t from_len;


	/*
	 * Create socket descriptor : "Domain=AF_INET","Type=SOCK_DGRAM","Protocol=0"
	 *  AF_INET= Connect with different machine, SOCK_DGRAM= UDP, 0=No additional protocol
	 *  */

	HAL_GPIO_WritePin(GPIOB, Red_LED_Pin, GPIO_PIN_SET); // turn on Red-LED

	socket_id = socket(AF_INET,SOCK_DGRAM,0);

//	Reset structure variable
	memset(&server_addr, 0, sizeof(server_addr));
	memset(&client_addr, 0, sizeof(client_addr));

// Initialize port
	server_addr.sin_family = AF_INET;	// Domain used in socket descriptor
	server_addr.sin_addr.s_addr = inet_addr(OWN_IP_ADDR); // Permit given IP Address
//	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	server_addr.sin_port = htons(OWN_PORT);	// Permit given Port

//	Bind the socket to a defined port
	if (bind(socket_id,(struct sockaddr *) &server_addr, sizeof(server_addr)) == -1){
		Error_Handler();
	}

	from_len = sizeof(client_addr);

//	Endlos loop
	while(1){
		HAL_GPIO_TogglePin(GPIOB, Red_LED_Pin);	// toggle blue led

//	Clear recv_data array
		memset(&from_udp_task.msg_buffer,0,BUFFER_LENGTH);
//	Receive data from server
		recvfrom(socket_id,from_udp_task.msg_buffer,sizeof(from_udp_task.msg_buffer),0,(struct sockaddr *)&client_addr,&from_len);

// Set flag according received message
		if(strcmp(from_udp_task.msg_buffer,"ECHO_ON") == 0){
//			HAL_GPIO_WritePin(GPIOB, Red_LED_Pin,GPIO_PIN_SET);
			echo_flag = TRUE;
		}
		else if(strcmp(from_udp_task.msg_buffer,"ECHO_OFF")== 0){
//			HAL_GPIO_WritePin(GPIOB, Red_LED_Pin,GPIO_PIN_RESET);
			echo_flag = FALSE;
		}

		if (echo_flag == TRUE){
			from_udp_task.addr = client_addr.sin_addr.s_addr;
			from_udp_task.port = client_addr.sin_port;
			/* Make string */
			sprintf(from_udp_task.msg_buffer, "%s\n", from_udp_task.msg_buffer);
			xQueueSend(Own_Queue, &from_udp_task, 10); // send to gatekeeper task
		}
	}
}

/* Gatekeeper task */
void vGateKeeper_Task(void *pvParameters)
{
	char stri1[] = "{vGateKeeper_Task} [Info]	Entered vGateKeeper_Task.\n\r";
	HAL_UART_Transmit(&huart3, (uint8_t*)stri1, strlen(stri1), HAL_MAX_DELAY);

	queue_struct recv_from; // struct variable to store received message from queue
	struct sockaddr_in client_addr; // struct variable for udp client

	tempVal_struct tempStruct;
	char s_tempVal[50];

	while(1){
		memset(&recv_from,0,sizeof(recv_from)); // reset the struct variable
		xQueueReceive(Own_Queue, &recv_from, portMAX_DELAY); // receive message

		/* UDP client initialization */
		client_addr.sin_family = AF_INET;
		client_addr.sin_addr.s_addr = recv_from.addr;
		client_addr.sin_port = recv_from.port;

		// Send Temperature data through UART
		xQueueReceive(temperatureVal_queue, &tempStruct, portMAX_DELAY); // receive message
		sprintf(s_tempVal,"Temp-Last-Val= %lu , Temp-Avr-Val= %lu\n\r", tempStruct.lastVal, tempStruct.avrVal);
		HAL_UART_Transmit(&huart3, (uint8_t*)s_tempVal, strlen(s_tempVal), HAL_MAX_DELAY);

		/* Send message via UDP interface */
		sendto(socket_id,recv_from.msg_buffer,sizeof(recv_from.msg_buffer),0,(struct sockaddr *)&client_addr,sizeof(client_addr));

//		char *hello = "Hello from server";

//		sendto(socket_id, (const char *)hello, strlen(hello), 0, (const struct sockaddr *) &client_addr, sizeof(client_addr));
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
//void StartDefaultTask(void const * argument)
//{
//  /* init code for LWIP */
//  MX_LWIP_Init();
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
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
