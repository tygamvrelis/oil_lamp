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
#include <string.h>
#include "usart.h"
#include "App/table.h"
#include "App/sensing.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BUF_SIZE (MAX_TABLE_IDX * sizeof(imu_data_t) + 2)
#define STATUS (BUF_SIZE - 2)
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
osTimerId StatusLEDTmrHandle;
osStaticTimerDef_t StatusLEDTmrControlBlock;
osTimerId CameraLEDTmrHandle;
osStaticTimerDef_t CameraLEDTmrControlBlock;
osMutexId TableLockHandle;
osStaticMutexDef_t TableLockControlBlock;
osSemaphoreId I2C2SemHandle;
osStaticSemaphoreDef_t I2C2SemControlBlock;
osSemaphoreId I2C1SemHandle;
osStaticSemaphoreDef_t I2C1SemControlBlock;
osSemaphoreId TxSemHandle;
osStaticSemaphoreDef_t TxSemControlBlock;
osSemaphoreId RxSemHandle;
osStaticSemaphoreDef_t RxSemControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/**
 * @brief Toggles the green LED, LD2
 * @note  Useful for debugging
 */
inline void toggle_status_led(){
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

/** @brief The states the camera synchronization LED can be in */
typedef enum{
    CAMERA_LED_ON = GPIO_PIN_SET,   /**< LED is on  */
	CAMERA_LED_OFF = GPIO_PIN_RESET /**< LED is off */
}CameraState_t;

/** @brief Turns the camera synchronization LED on or off */
inline void set_camera_led_state(CameraState_t state){
    HAL_GPIO_WritePin(CAMERA_LED_GPIO_Port, CAMERA_LED_Pin, state);
}

/** @brief Reads the camera LED pin to see whether it's on or off */
inline GPIO_PinState get_camera_led_state(){
    return HAL_GPIO_ReadPin(CAMERA_LED_GPIO_Port, CAMERA_LED_Pin);
}
/* USER CODE END FunctionPrototypes */

void StartRxTask(void const * argument);
void StartTxTask(void const * argument);
void StartImuBaseTask(void const * argument);
void StartImuLampTask(void const * argument);
void StatusLEDTmrCallback(void const * argument);
void CameraLEDTmrCallback(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];
  
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )  
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}                   
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of TableLock */
  osMutexStaticDef(TableLock, &TableLockControlBlock);
  TableLockHandle = osMutexCreate(osMutex(TableLock));

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

  /* definition and creation of TxSem */
  osSemaphoreStaticDef(TxSem, &TxSemControlBlock);
  TxSemHandle = osSemaphoreCreate(osSemaphore(TxSem), 1);

  /* definition and creation of RxSem */
  osSemaphoreStaticDef(RxSem, &RxSemControlBlock);
  RxSemHandle = osSemaphoreCreate(osSemaphore(RxSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of StatusLEDTmr */
  osTimerStaticDef(StatusLEDTmr, StatusLEDTmrCallback, &StatusLEDTmrControlBlock);
  StatusLEDTmrHandle = osTimerCreate(osTimer(StatusLEDTmr), osTimerPeriodic, NULL);

  /* definition and creation of CameraLEDTmr */
  osTimerStaticDef(CameraLEDTmr, CameraLEDTmrCallback, &CameraLEDTmrControlBlock);
  CameraLEDTmrHandle = osTimerCreate(osTimer(CameraLEDTmr), osTimerOnce, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  // Make status LED blink at 0.5 Hz
  osTimerStop(StatusLEDTmrHandle);
  osTimerStart(StatusLEDTmrHandle, 1000);
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
	uint8_t status;
	uint8_t buf[BUF_SIZE] = {0};
	buf[BUF_SIZE - 1] = '\n'; // Termination character
	for (;;)
	{
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TX_PERIOD_MS));

		// Pack data
		for (uint8_t i = 0; i < MAX_TABLE_IDX; ++i)
		{
			read_table(i, (float*)&buf[i * sizeof(imu_data_t)], sizeof(imu_data_t));
		}

		// Add camera synch flag if needed
		status = 0;
		status |= get_camera_led_state();
		buf[STATUS] = status;

		// Send
        if (HAL_UART_Transmit_DMA(&huart2, buf, sizeof(buf)) != HAL_OK)
        {
            HAL_UART_AbortTransmit_IT(&huart2);
        }
        else
        {
        	xSemaphoreTake(TxSemHandle, pdMS_TO_TICKS(3));
        }
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
	attachSemaphore(&imu_base, I2C1SemHandle);
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
	osDelay(TX_PERIOD_MS - 2);
    imu_data_t base_data;
	for(;;)
	{
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(IMU_CYCLE_MS));

        accelReadIT(&imu_base);
        gyroReadIT(&imu_base);
        base_data = get_data(&imu_base);
        write_table(TABLE_IDX_BASE_DATA, (float*)&base_data, sizeof(imu_data_t));
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
	attachSemaphore(&imu_lamp, I2C2SemHandle);
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    osDelay(TX_PERIOD_MS - 2);
    imu_data_t lamp_data;
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(IMU_CYCLE_MS));

        accelReadIT(&imu_lamp);
        gyroReadIT(&imu_lamp);
        lamp_data = get_data(&imu_lamp);
        write_table(TABLE_IDX_LAMP_DATA, (float*)&lamp_data, sizeof(imu_data_t));
    }
  /* USER CODE END StartImuLampTask */
}

/* StatusLEDTmrCallback function */
void StatusLEDTmrCallback(void const * argument)
{
  /* USER CODE BEGIN StatusLEDTmrCallback */
	toggle_status_led();
  /* USER CODE END StatusLEDTmrCallback */
}

/* CameraLEDTmrCallback function */
void CameraLEDTmrCallback(void const * argument)
{
  /* USER CODE BEGIN CameraLEDTmrCallback */
	set_camera_led_state(CAMERA_LED_OFF);
  /* USER CODE END CameraLEDTmrCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == B1_Pin)
	{
		// Blue button on Nucleo was pushed.
		// Turn on camera synchronization LED for 100 ms (about 3 frames...?)
		set_camera_led_state(CAMERA_LED_ON);
        osTimerStop(CameraLEDTmrHandle);
        osTimerStart(CameraLEDTmrHandle, 100);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (huart->Instance == USART2 && TxSemHandle != NULL){
    	xSemaphoreGiveFromISR(TxSemHandle, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (huart->Instance == USART2 && RxSemHandle != NULL){
    	xSemaphoreGiveFromISR(RxSemHandle, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	// Returns the semaphore taken after non-blocking reception ends
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (hi2c->Instance == I2C2){
    	xSemaphoreGiveFromISR(I2C2SemHandle, &xHigherPriorityTaskWoken);
	}
	else if (hi2c->Instance == I2C1){
    	xSemaphoreGiveFromISR(I2C1SemHandle, &xHigherPriorityTaskWoken);
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c){
	// Returns the semaphore taken after non-blocking transmission ends
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if (hi2c->Instance == I2C2){
    	xSemaphoreGiveFromISR(I2C2SemHandle, &xHigherPriorityTaskWoken);
	}
	else if (hi2c->Instance == I2C1){
    	xSemaphoreGiveFromISR(I2C1SemHandle, &xHigherPriorityTaskWoken);
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
