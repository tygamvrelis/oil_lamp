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
#include <stdbool.h>
#include <string.h>
#include "usart.h"
#include "wwdg.h"
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
#define BUF_SIZE     (2 + MAX_TABLE_IDX * sizeof(imu_data_t) + 2)
#define BUF_HEADER_1 0
#define BUF_HEADER_2 1
#define BUF_IMU      2
#define BUF_STATUS   (BUF_SIZE - 2)
#define BUF_FOOTER   (BUF_SIZE - 1)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static bool scheduler_has_started = false;
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
osSemaphoreId LampSemHandle;
osStaticSemaphoreDef_t LampSemControlBlock;
osSemaphoreId BaseSemHandle;
osStaticSemaphoreDef_t BaseSemControlBlock;
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
    CAMERA_LED_OFF = GPIO_PIN_RESET, /**< LED is off */
    CAMERA_LED_ON = GPIO_PIN_SET     /**< LED is on  */
}CameraState_t;

/** @brief Turns the camera synchronization LED on or off */
inline void set_camera_led_state(CameraState_t state){
    HAL_GPIO_WritePin(CAMERA_LED_GPIO_Port, CAMERA_LED_Pin, state);
}

/** @brief Reads the camera LED pin to see whether it's on or off */
inline GPIO_PinState get_camera_led_state(){
    return HAL_GPIO_ReadPin(CAMERA_LED_GPIO_Port, CAMERA_LED_Pin);
}

inline void blink_camera_led()
{
    // Turn on camera synchronization LED for 100 ms (about 3 frames...?)
    set_camera_led_state(CAMERA_LED_ON);
    osTimerStart(CameraLEDTmrHandle, 100);
}

// TODO(tyler): move this somewhere more suitable
typedef struct{
    const uint8_t size;
    uint8_t iHead;
    uint8_t iTail;
    uint8_t* pBuff;
}CircBuff_t;

uint8_t pop(CircBuff_t* buff)
{
    uint8_t data = buff->pBuff[buff->iTail];
    ++buff->iTail;
    if (buff->iTail == buff->size)
    {
        buff->iTail = 0;
    }
    return data;
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
  /* definition and creation of LampSem */
  osSemaphoreStaticDef(LampSem, &LampSemControlBlock);
  LampSemHandle = osSemaphoreCreate(osSemaphore(LampSem), 1);

  /* definition and creation of BaseSem */
  osSemaphoreStaticDef(BaseSem, &BaseSemControlBlock);
  BaseSemHandle = osSemaphoreCreate(osSemaphore(BaseSem), 1);

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
  osThreadStaticDef(ImuBase, StartImuBaseTask, osPriorityNormal, 0, 128, ImuBaseTaskBuffer, &ImuBaseTaskControlBlock);
  ImuBaseHandle = osThreadCreate(osThread(ImuBase), NULL);

  /* definition and creation of ImuLamp */
  osThreadStaticDef(ImuLamp, StartImuLampTask, osPriorityNormal, 0, 128, ImuLampTaskBuffer, &ImuLampTaskControlBlock);
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
    scheduler_has_started = true;
    MX_WWDG_Init();
    const uint32_t RX_CYCLE_TIME = osKernelSysTickMicroSec(1000);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    static uint8_t rx_buff[32] = {0}; // static => no stack usage
    CircBuff_t circ_buff = {sizeof(rx_buff), 0, 0, rx_buff};

    HAL_UART_Receive_DMA(&huart2, rx_buff, sizeof(rx_buff));
    for(;;)
    {
        osDelayUntil(&xLastWakeTime, RX_CYCLE_TIME);
        circ_buff.iHead = circ_buff.size - huart2.hdmarx->Instance->NDTR;
        while(circ_buff.iHead != circ_buff.iTail){
            // Got data, do something with it
            uint8_t data = pop(&circ_buff);
            if ((char)data == 'L')
            {
                blink_camera_led();
            }
        }
        if (huart2.RxState == HAL_UART_STATE_ERROR)
        {
            HAL_UART_AbortReceive(&huart2);
            HAL_UART_Receive_DMA(&huart2, rx_buff, sizeof(rx_buff));
        }
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
    const uint32_t TX_CYCLE_TIME = osKernelSysTickMicroSec(TX_PERIOD_MS * 1000);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t status;
    uint8_t buf[BUF_SIZE] = {0};
    buf[BUF_HEADER_1] = 0xAA; // 0b10101010
    buf[BUF_HEADER_2] = 0xAA;
    buf[BUF_FOOTER] = '\n'; // Termination character
    for (;;)
    {
        osDelayUntil(&xLastWakeTime, TX_CYCLE_TIME);
        HAL_WWDG_Refresh(&hwwdg);

        // Pack data
        for (uint8_t i = 0; i < MAX_TABLE_IDX; ++i)
        {
            read_table(i, (float*)&buf[BUF_IMU + i * sizeof(imu_data_t)], sizeof(imu_data_t));
        }

        // Add camera synch flag if needed
        status = 0;
        status |= get_camera_led_state();
        buf[BUF_STATUS] = status;

        // Send
        if (HAL_UART_Transmit_DMA(&huart2, buf, sizeof(buf)) != HAL_OK)
        {
            HAL_UART_AbortTransmit_IT(&huart2);
        }
        else
        {
            // BUF_SIZE = 52 bytes = 52*8 = 416 bits
            // 115200 symbols/sec => 115200*8/10 = 92160 bps
            // 416 bits / 92160 bps = 4.55 ms
            xSemaphoreTake(TxSemHandle, pdMS_TO_TICKS(5));
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
    const uint32_t IMU_CYCLE_TIME = osKernelSysTickMicroSec(IMU_CYCLE_MS * 1000);
    attachSemaphore(&imu_base, BaseSemHandle);
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    osDelay(TX_PERIOD_MS - 2);
    imu_data_t base_data;
    for(;;)
    {
        osDelayUntil(&xLastWakeTime, IMU_CYCLE_TIME);
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
    const uint32_t IMU_CYCLE_TIME = osKernelSysTickMicroSec(IMU_CYCLE_MS * 1000);
    attachSemaphore(&imu_lamp, LampSemHandle);
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    osDelay(TX_PERIOD_MS - 2);
    imu_data_t lamp_data;
    for(;;)
    {
        osDelayUntil(&xLastWakeTime, IMU_CYCLE_TIME);
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
void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg)
{
//    // Watchdog wants to reset
//    // Check if I2C is frozen, if so, try to software-reset it. Otherwise, let
//    // the reset occur.
    bool reset_lamp = false, reset_base = false;
    static uint32_t last_tick_entry = 0;
    uint32_t cur_tick = HAL_GetTick();
    if (imu_lamp.hi2c->State != HAL_I2C_STATE_READY ||
        __HAL_I2C_GET_FLAG(imu_lamp.hi2c, I2C_FLAG_BUSY) != RESET)
    {
        SET_BIT(imu_lamp.hi2c->Instance->CR1, I2C_CR1_SWRST);
        asm("nop");
        CLEAR_BIT(imu_lamp.hi2c->Instance->CR1, I2C_CR1_SWRST);
        asm("nop");
        reset_lamp = true;
    }
    if (imu_base.hi2c->State != HAL_I2C_STATE_READY ||
        __HAL_I2C_GET_FLAG(imu_base.hi2c, I2C_FLAG_BUSY) != RESET)
    {
        SET_BIT(imu_base.hi2c->Instance->CR1, I2C_CR1_SWRST);
        asm("nop");
        CLEAR_BIT(imu_base.hi2c->Instance->CR1, I2C_CR1_SWRST);
        asm("nop");
        reset_base = true;
    }
    if ((cur_tick - last_tick_entry > 20) && (reset_lamp || reset_base))
    {
        // Seems like an I2C driver issue was causing the blockage. As long as
        // this is happening infrequently, the SWRST above should recover it.
        // The acceptable time difference above must be greater than the Watchdog
        // time; otherwise, it could mean that we are only here again because
        // the previous recovery attempt did not work.
        HAL_WWDG_Refresh(&hwwdg);
    }
    last_tick_entry = cur_tick;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (scheduler_has_started)
    {
        if (GPIO_Pin == B1_Pin)
        {
            // Blue button on Nucleo was pushed
            blink_camera_led();
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if (scheduler_has_started)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (huart->Instance == USART2 && TxSemHandle != NULL){
            xSemaphoreGiveFromISR(TxSemHandle, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (scheduler_has_started)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (huart->Instance == USART2 && RxSemHandle != NULL){
            xSemaphoreGiveFromISR(RxSemHandle, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
    if (scheduler_has_started)
    {
        // Returns the semaphore taken after non-blocking reception ends
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (hi2c->Instance == I2C3){
            xSemaphoreGiveFromISR(LampSemHandle, &xHigherPriorityTaskWoken);
        }
        else if (hi2c->Instance == I2C1){
            xSemaphoreGiveFromISR(BaseSemHandle, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c){
    if (scheduler_has_started)
    {
        // Returns the semaphore taken after non-blocking transmission ends
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (hi2c->Instance == I2C3){
            xSemaphoreGiveFromISR(LampSemHandle, &xHigherPriorityTaskWoken);
        }
        else if (hi2c->Instance == I2C1){
            xSemaphoreGiveFromISR(BaseSemHandle, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
