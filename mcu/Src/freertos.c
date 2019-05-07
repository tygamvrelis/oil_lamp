/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "usart.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId RxHandle;
uint32_t RxTaskBuffer[ 128 ];
osStaticThreadDef_t RxTaskControlBlock;
osThreadId TxHandle;
uint32_t TxTaskBuffer[ 128 ];
osStaticThreadDef_t TxTaskControlBlock;
osThreadId ImuBaseHandle;
uint32_t ImuBaseTaskBuffer[ 128 ];
osStaticThreadDef_t ImuBaseTaskControlBlock;
osThreadId ImuLampHandle;
uint32_t ImuLampTaskBuffer[ 128 ];
osStaticThreadDef_t ImuLampTaskControlBlock;
osSemaphoreId I2C2SemHandle;
osStaticSemaphoreDef_t I2C2SemControlBlock;
osSemaphoreId I2C1SemHandle;
osStaticSemaphoreDef_t I2C1SemControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartRxTask(void const * argument);
void StartTxTask(void const * argument);
void StartImuBaseTask(void const * argument);
void StartImuLampTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of I2C2Sem */
  osSemaphoreStaticDef(I2C2Sem, &I2C2SemControlBlock);
  I2C2SemHandle = osSemaphoreCreate(osSemaphore(I2C2Sem), 1);

  /* definition and creation of I2C1Sem */
  osSemaphoreStaticDef(I2C1Sem, &I2C1SemControlBlock);
  I2C1SemHandle = osSemaphoreCreate(osSemaphore(I2C1Sem), 1);

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
  /* definition and creation of Rx */
  osThreadStaticDef(Rx, StartRxTask, osPriorityRealtime, 0, 128, RxTaskBuffer, &RxTaskControlBlock);
  RxHandle = osThreadCreate(osThread(Rx), NULL);

  /* definition and creation of Tx */
  osThreadStaticDef(Tx, StartTxTask, osPriorityHigh, 0, 128, TxTaskBuffer, &TxTaskControlBlock);
  TxHandle = osThreadCreate(osThread(Tx), NULL);

  /* definition and creation of ImuBase */
  osThreadStaticDef(ImuBase, StartImuBaseTask, osPriorityAboveNormal, 0, 128, ImuBaseTaskBuffer, &ImuBaseTaskControlBlock);
  ImuBaseHandle = osThreadCreate(osThread(ImuBase), NULL);

  /* definition and creation of ImuLamp */
  osThreadStaticDef(ImuLamp, StartImuLampTask, osPriorityAboveNormal, 0, 128, ImuLampTaskBuffer, &ImuLampTaskControlBlock);
  ImuLampHandle = osThreadCreate(osThread(ImuLamp), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartRxTask */
/**
  * @brief  Function implementing the Rx thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartRxTask */
void StartRxTask(void const * argument)
{

  /* USER CODE BEGIN StartRxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartRxTask */
}

/* USER CODE BEGIN Header_StartTxTask */
/**
* @brief Function implementing the Tx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTxTask */
void StartTxTask(void const * argument)
{
  /* USER CODE BEGIN StartTxTask */
	// For packet timing management
	TickType_t xLastWakeTime = xTaskGetTickCount();
	TickType_t TX_PERIOD_MS = 10;
	uint8_t buf[2] = {};
	for (;;) {
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TX_PERIOD_MS));

		// TODO: pack data and transmit
		HAL_UART_Transmit_DMA(&huart2, buf, sizeof(buf));
	}
  /* USER CODE END StartTxTask */
}

/* USER CODE BEGIN Header_StartImuBaseTask */
/**
* @brief Function implementing the ImuBase thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartImuBaseTask */
void StartImuBaseTask(void const * argument)
{
  /* USER CODE BEGIN StartImuBaseTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartImuBaseTask */
}

/* USER CODE BEGIN Header_StartImuLampTask */
/**
* @brief Function implementing the ImuLamp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartImuLampTask */
void StartImuLampTask(void const * argument)
{
  /* USER CODE BEGIN StartImuLampTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartImuLampTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
