/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "semphr.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	GPIO_TypeDef* port;
	uint16_t pin;
	TickType_t time;
}led_t;

typedef struct {
    uint16_t valor;
    float tensao;
    uint32_t timestamp;
}ecg_data_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ECG_BUFFER_SIZE 512
#define ECG_SAMPLE_RATE_HZ 250
#define ECG_SAMPLE_PERIOD_MS (1000 / ECG_SAMPLE_RATE_HZ)
#define ADC_VREF 3.3f
#define ADC_RESOLUTION 4096.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
SemaphoreHandle_t bSemaphore;
SemaphoreHandle_t lSemaphore;
SemaphoreHandle_t sUART;
SemaphoreHandle_t mUART;
SemaphoreHandle_t adcSemaphore;
QueueHandle_t ecgQueue;

TaskHandle_t xecg_task = NULL;
TaskHandle_t xuart_task = NULL;

char sinal = 0;
uint8_t *Sstring = (uint8_t *) "Sensor desligado!\n\r";

static ecg_data_t ecg_buffer[ECG_BUFFER_SIZE];
static uint16_t buffer_head = 0;
static uint16_t buffer_tail = 0;
static uint16_t buffer_count = 0;

volatile uint16_t adc_value = 0;
volatile uint8_t adc_conversion_complete = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);

static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
void ecg_task(void *args);
void process_ecg_data(ecg_data_t *data);
void add_to_ecg_buffer(ecg_data_t *data);
float apply_simple_filter(float input);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void led_task(void *args){
	led_t *led = (led_t *) args;

	while(1){
		if(xSemaphoreTake(lSemaphore,portMAX_DELAY)==pdTRUE){
			HAL_GPIO_TogglePin(led->port, led->pin);
		}
	}
}

void button_task(void *args){

	while(1){

		if(xSemaphoreTake(bSemaphore,portMAX_DELAY)==pdTRUE){
			vTaskDelay(50);
			xSemaphoreGive(lSemaphore);
			if(sinal==1){
				sinal=0;
				Sstring = (uint8_t *) "Sensor desligado!\n\r";
				vTaskSuspend(xecg_task);
				HAL_TIM_Base_Stop_IT(&htim3);
			}else{
				sinal=1;
				Sstring = (uint8_t *) "Sensor ligado!\n\r";
				vTaskResume(xuart_task);
				vTaskResume(xecg_task);
				HAL_TIM_Base_Start_IT(&htim3);
			}
			HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
		}
	}
}

void uart_task(void *argument){
	char uart_buffer[100];
	ecg_data_t ecg_data;

	while(1){

		if(sinal==0){
			if(xSemaphoreTake(mUART,portMAX_DELAY)== pdTRUE){
				HAL_UART_Transmit_IT(&hlpuart1, Sstring, 19);
				xSemaphoreTake(sUART, portMAX_DELAY);
				xSemaphoreGive(mUART);
				vTaskSuspend(xuart_task);
			}
		}else{
			if(xQueueReceive(ecgQueue, &ecg_data, pdMS_TO_TICKS(100)) == pdPASS){
				if(xSemaphoreTake(mUART,portMAX_DELAY)== pdTRUE){
					int len = snprintf(uart_buffer, sizeof(uart_buffer),"ECG: %d, %.3fV, %lu ms\n\r",ecg_data.valor, ecg_data.tensao, ecg_data.timestamp);
					HAL_UART_Transmit_IT(&hlpuart1, (uint8_t*)uart_buffer, len);
					xSemaphoreTake(sUART, portMAX_DELAY);
					xSemaphoreGive(mUART);
				}
			}
		}
	}
}

void ecg_task(void *args){
	ecg_data_t ecg_data;
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(ECG_SAMPLE_PERIOD_MS);

	xLastWakeTime = xTaskGetTickCount();

	while(1){
		// Aguarda sinal do timer para fazer amostragem
		if(xSemaphoreTake(adcSemaphore, portMAX_DELAY) == pdTRUE){
			// Inicia conversão ADC
			HAL_ADC_Start(&hadc1);

			// Aguarda conversão completar
			if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK){
				// Lê o valor do ADC
				ecg_data.valor = HAL_ADC_GetValue(&hadc1);
				ecg_data.tensao = (ecg_data.valor * ADC_VREF) / ADC_RESOLUTION;
				ecg_data.timestamp = xTaskGetTickCount();

				// Aplica filtro simples
				ecg_data.tensao = apply_simple_filter(ecg_data.tensao);

				// Adiciona ao buffer circular
				add_to_ecg_buffer(&ecg_data);

				// Envia dados para a fila (não bloqueia se fila estiver cheia)
				xQueueSend(ecgQueue, &ecg_data, 0);

				// Processa os dados (detecção de picos, etc.)
				process_ecg_data(&ecg_data);
			}

			HAL_ADC_Stop(&hadc1);
		}
	}
}

float apply_simple_filter(float input){
	static float filter_buffer[8] = {0};
	static uint8_t filter_index = 0;
	float sum = 0;

	filter_buffer[filter_index] = input;
	filter_index = (filter_index + 1) % 8;

	for(int i = 0; i < 8; i++){
		sum += filter_buffer[i];
	}

	return sum / 8.0f;
}

void add_to_ecg_buffer(ecg_data_t *data){
	ecg_buffer[buffer_head] = *data;
	buffer_head = (buffer_head + 1) % ECG_BUFFER_SIZE;

	if(buffer_count < ECG_BUFFER_SIZE){
		buffer_count++;
	}else{
		buffer_tail = (buffer_tail + 1) % ECG_BUFFER_SIZE;
	}
}

void process_ecg_data(ecg_data_t *data){
	// Aqui você pode implementar:
	// - Detecção de picos R
	// - Cálculo de BPM
	// - Análise de variabilidade
	// - Detecção de arritmias

	// Exemplo simples: detecta se o sinal está acima de um threshold
	static float threshold = 1.8f; // Ajuste conforme necessário
	static uint32_t last_peak_time = 0;

	if(data->tensao > threshold){
		uint32_t current_time = data->timestamp;
		if(current_time - last_peak_time > pdMS_TO_TICKS(300)){ // Evita dupla detecção
			last_peak_time = current_time;
			// Pico detectado - pode calcular BPM aqui
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	signed portBASE_TYPE pxHigherPriorityTaskWokenTX = pdFALSE;

	xSemaphoreGiveFromISR(sUART, &pxHigherPriorityTaskWokenTX);

	if (pxHigherPriorityTaskWokenTX == pdTRUE)
	{
		portYIELD();
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	static led_t led = {.port = GPIOA, .pin = GPIO_PIN_5, .time = 500};
	//(void)xTaskCreate(button_task, "button_task", 1024, NULL, 7, NULL);

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
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  bSemaphore = xSemaphoreCreateBinary();
  lSemaphore = xSemaphoreCreateBinary();
  sUART = xSemaphoreCreateBinary();
  mUART = xSemaphoreCreateMutex();
  adcSemaphore = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  ecgQueue = xQueueCreate(32, sizeof(ecg_data_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
<<<<<<< Updated upstream
//  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
//  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
=======
>>>>>>> Stashed changes

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	(void)xTaskCreate(led_task, "led_task", 128, &led, 2, NULL);
	(void)xTaskCreate(uart_task, "uart_task", 128, NULL, 1, &xuart_task);
	(void)xTaskCreate(button_task, "button_task", 128, NULL, 3, NULL);
	(void)xTaskCreate(ecg_task, "ecg_task", 256, NULL, 4, &xecg_task);
  /* USER CODE END RTOS_THREADS */

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (GPIO_Pin == Button_Pin)
	{
		HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
		xSemaphoreGiveFromISR(bSemaphore, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
<<<<<<< Updated upstream
//void StartDefaultTask(void const * argument)
//{
//  /* USER CODE BEGIN 5 */
//  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
//  /* USER CODE END 5 */
//}
=======
>>>>>>> Stashed changes

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM3)
  {
	  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	  // Sinaliza para a task do ECG fazer uma amostragem
	  xSemaphoreGiveFromISR(adcSemaphore, &xHigherPriorityTaskWoken);
	  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
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
