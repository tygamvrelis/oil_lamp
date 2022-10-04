/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "App/util.h"
#include "App/table.h"
#include "App/rx.h"
#include "App/Notification.h"
#include "LSS/lss.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BUF_SIZE     (2 + TABLE_IDX_MAX_SENSOR_DATA + 2)
#define BUF_HEADER_1 0
#define BUF_HEADER_2 1
#define BUF_IMU      2
#define BUF_LAMP_IMU (2 + sizeof(imu_data_t))
#define BUF_BASE_IMU 2
#define BUF_STATUS   (BUF_SIZE - 2)
#define BUF_FOOTER   (BUF_SIZE - 1)

#define PC_UART huart2
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static bool scheduler_has_started = false;
/* USER CODE END Variables */
osThreadId TxHandle;
uint32_t TxTaskBuffer[ 128 ];
osStaticThreadDef_t TxTaskControlBlock;
osThreadId RxHandle;
uint32_t RxTaskBuffer[ 128 ];
osStaticThreadDef_t RxTaskControlBlock;
osThreadId ControlHandle;
uint32_t ControlTaskBuffer[ 512 ];
osStaticThreadDef_t ControlTaskControlBlock;
osTimerId StatusLEDTmrHandle;
osStaticTimerDef_t StatusLEDTmrControlBlock;
osTimerId CameraLEDTmrHandle;
osStaticTimerDef_t CameraLEDTmrControlBlock;
osMutexId TableLockHandle;
osStaticMutexDef_t TableLockControlBlock;
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

#define NOTIFY_CLEAR 0
#define NOTIFY_THREAD_DI 1
/** @brief Enables the control (servos) thread */
void enable_control()
{
    xTaskNotify((TaskHandle_t)ControlHandle, NOTIFY_CLEAR, eSetValueWithOverwrite);
}

/** @brief Disables the control (servos) thread */
void disable_control()
{
    xTaskNotify((TaskHandle_t)ControlHandle, NOTIFY_THREAD_DI, eSetBits);
}

/** @brief Returns true if the current thread is enabled, otherwise false */
bool thread_is_enabled(){
    uint32_t ulNotifiedValue;
    // Don't clear bits and don't block
    xTaskNotifyWait(0, 0, &ulNotifiedValue, 0);
    return (ulNotifiedValue & NOTIFY_THREAD_DI) != NOTIFY_THREAD_DI;
}
/* USER CODE END FunctionPrototypes */

void StartTxTask(void const * argument);
void StartRxTask(void const * argument);
void StartControlTask(void const * argument);
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
  /* definition and creation of Tx */
  osThreadStaticDef(Tx, StartTxTask, osPriorityHigh, 0, 128, TxTaskBuffer, &TxTaskControlBlock);
  TxHandle = osThreadCreate(osThread(Tx), NULL);

  /* definition and creation of Rx */
  osThreadStaticDef(Rx, StartRxTask, osPriorityRealtime, 0, 128, RxTaskBuffer, &RxTaskControlBlock);
  RxHandle = osThreadCreate(osThread(Rx), NULL);

  /* definition and creation of Control */
  osThreadStaticDef(Control, StartControlTask, osPriorityNormal, 0, 512, ControlTaskBuffer, &ControlTaskControlBlock);
  ControlHandle = osThreadCreate(osThread(Control), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
    static const uint32_t TX_CYCLE_TIME = osKernelSysTickMicroSec(TX_PERIOD_MS * 1000);

    uint8_t status;
    uint8_t buf[BUF_SIZE] = {0};
    buf[BUF_HEADER_1] = 0xAA; // 0b10101010
    buf[BUF_HEADER_2] = 0xAA;
    buf[BUF_FOOTER] = '\n'; // Termination character

    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        osDelayUntil(&xLastWakeTime, TX_CYCLE_TIME);
        HAL_WWDG_Refresh(&hwwdg);

        // Pack data
        read_table(TABLE_IDX_BASE_DATA, (uint8_t*)&buf[BUF_BASE_IMU], sizeof(imu_data_t));
        read_table(TABLE_IDX_LAMP_DATA, (uint8_t*)&buf[BUF_LAMP_IMU], sizeof(imu_data_t));

        // Add camera synch flag if needed
        status = 0;
        status |= get_camera_led_state();
        buf[BUF_STATUS] = status;

        // Send
        if (HAL_UART_Transmit_DMA(&PC_UART, buf, sizeof(buf)) != HAL_OK)
        {
            HAL_UART_AbortTransmit_IT(&PC_UART);
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
    static const uint32_t RX_CYCLE_TIME = osKernelSysTickMicroSec(1000);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    static uint8_t rx_buff[32] = {0}; // static => no stack usage
    static uint8_t cmd_buff[16] = {0};
    CircBuff_t circ_buff = {sizeof(rx_buff), 0, 0, rx_buff};
    uint8_t cnt = 0; // number of bytes received so far
    uint8_t num_needed = 0; // length of command packet currently being recv

    enum {
        CMD_NONE,
        CMD_BLINK = 'L',
        CMD_CTRL_DI = '0',
        CMD_CTRL_EN = '1',
        CMD_SENS_DI = '2',
        CMD_SENS_EN = '3',
        CMD_ZERO_REF = 'Z',
        CMD_ZERO_REF_OUTER = 'O',
        CMD_ZERO_REF_INNER = 'I',
        CMD_RESET = 'R',
        CMD_ANGLE = 'A',
        CMD_ANGLE_OUTER = '8', // Exact value doesn't matter, just has to be
        CMD_ANGLE_INNER = '9', // unique (these are used as parse states only)
        CMD_CHECKSUM = 'C'     // same
    };

    MX_WWDG_Init();
    enable_control();
    HAL_UART_Receive_DMA(&PC_UART, rx_buff, sizeof(rx_buff));
    for(;;)
    {
        osDelayUntil(&xLastWakeTime, RX_CYCLE_TIME);
        circ_buff.iHead = circ_buff.size - PC_UART.hdmarx->Instance->CNDTR;
        while(circ_buff.iHead != circ_buff.iTail)
        {
            uint8_t data = pop(&circ_buff);
            if (num_needed == 0)
            {
                // If we are not in the middle of receiving a command packet...
                // then enter the state where we are waiting for the full thing
                // to arrive
                if ((char)data == (char)CMD_BLINK)
                {
                    num_needed = 2;
                }
                else if ((char)data == (char)CMD_RESET)
                {
                    num_needed = 2;
                }
                else if ((char)data == (char)CMD_ANGLE)
                {
                    num_needed = 1 + sizeof(float) * 2 + 1;
                }
                else if ((char)data == (char)CMD_ZERO_REF)
                {
                    num_needed = 1 + sizeof(float) * 2 + 1;
                }
                else if ((char)data == (char)CMD_CTRL_DI)
                {
                    num_needed = 2;
                }
                else if ((char)data == (char)CMD_CTRL_EN)
                {
                    num_needed = 2;
                }
                else if ((char)data == (char)CMD_SENS_DI)
                {
                    num_needed = 2;
                }
                else if ((char)data == (char)CMD_SENS_EN)
                {
                    num_needed = 2;
                }
            }

            if (num_needed != 0)
            {
                // If we are in the process of receiving a command packet...
                cmd_buff[cnt] = data;
                ++cnt;
            }

            if (num_needed != 0 && cnt == num_needed)
            {
                // Verify integrity
                uint8_t check = rs232_checksum(cmd_buff, num_needed);
                if (check == 0)
                {
                    // Perform requested command function
                    uint8_t cmd = cmd_buff[0];
                    switch(cmd)
                    {
                        case CMD_BLINK:
                            blink_camera_led();
                            break;
                        case CMD_RESET:
                            taskENTER_CRITICAL();
                            NVIC_SystemReset();
                            break;
                        case CMD_CTRL_DI:
                            disable_control();
                            break;
                        case CMD_CTRL_EN:
                            enable_control();
                            break;
                        case CMD_SENS_DI:
//                            disable_sensing();
                            break;
                        case CMD_SENS_EN:
//                            enable_sensing();
                            break;
                        case CMD_ANGLE:
                            // Outer
                            write_table(
                                TABLE_IDX_OUTER_GIMBAL_ANGLE,
                                &cmd_buff[1],
                                sizeof(float)
                            );

                            // Inner
                            write_table(
                                TABLE_IDX_INNER_GIMBAL_ANGLE,
                                &cmd_buff[1 + sizeof(float)],
                                sizeof(float)
                            );
                            break;
                        case CMD_ZERO_REF:
                            // Outer
                            write_table(
                                TABLE_IDX_ZERO_REF_OUTER_GIMBAL,
                                &cmd_buff[1],
                                sizeof(float)
                            );

                            // Inner
                            write_table(
                                TABLE_IDX_ZERO_REF_INNER_GIMBAL,
                                &cmd_buff[1 + sizeof(float)],
                                sizeof(float)
                            );
                            break;
                        default:
                            break;
                    }
                }
                num_needed = 0;
                cnt = 0;
            }
        }
        if (PC_UART.RxState == HAL_UART_STATE_ERROR || PC_UART.ErrorCode != HAL_UART_ERROR_NONE)
        {
            HAL_UART_AbortReceive(&PC_UART);
            HAL_UART_Receive_DMA(&PC_UART, rx_buff, sizeof(rx_buff));
        }
        // TODO: add timeout...?
    }
  /* USER CODE END StartRxTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlTask */
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */
    while (!thread_is_enabled())
    {
        continue;
    }

    static const uint32_t CONTROL_CYCLE_TIME = osKernelSysTickMicroSec(CONTROL_CYCLE_MS * 1000);
    static const float MIN_GIMBAL_ANGLE = -40.0;
    static const float MAX_GIMBAL_ANGLE = 40.0;
    float a_outer, a_inner, zero_ref_outer, zero_ref_inner;

    lss_set_io_type(IO_DMA);

    const int8_t ANGULAR_STIFFNESS = -4;
    const int8_t HOLDING_STIFFNESS = -4;
    const uint8_t OUTER_ID = 0;
    const float OUTER_OFFSET = -102;
    const uint16_t AA = 1000;
    const uint16_t AD = 1000;
    lss_t servo_outer = {OUTER_ID, &huart1, OUTER_OFFSET};
    lss_set_led(&servo_outer, LSS_OFF);
    lss_toggle_motion_ctrl(&servo_outer, LSS_EM0);
    lss_set_speed(&servo_outer, 180.0);
    lss_set_aa(&servo_outer, AA);
    lss_set_ad(&servo_outer, AD);
    lss_set_as(&servo_outer, ANGULAR_STIFFNESS);
    lss_set_hs(&servo_outer, HOLDING_STIFFNESS);

    const uint8_t INNER_ID = 1;
    const float INNER_OFFSET = 49;
    lss_t servo_inner = {INNER_ID, &huart1, INNER_OFFSET};
    lss_set_led(&servo_inner, LSS_OFF);
    lss_toggle_motion_ctrl(&servo_inner, LSS_EM0);
    lss_set_speed(&servo_inner, 180.0);
    lss_set_aa(&servo_inner, AA);
    lss_set_ad(&servo_inner, AD);
    lss_set_as(&servo_inner, ANGULAR_STIFFNESS);
    lss_set_hs(&servo_inner, HOLDING_STIFFNESS);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        osDelayUntil(&xLastWakeTime, CONTROL_CYCLE_TIME);
        if (!thread_is_enabled())
        {
            continue;
        }

        read_table(TABLE_IDX_OUTER_GIMBAL_ANGLE, (uint8_t*)&a_outer, sizeof(float));
        read_table(TABLE_IDX_INNER_GIMBAL_ANGLE, (uint8_t*)&a_inner, sizeof(float));

        // Make sure we don't move the servos to angles outside these bounds
        a_outer = bound_float(a_outer, MIN_GIMBAL_ANGLE, MAX_GIMBAL_ANGLE);
        a_inner = bound_float(a_inner, MIN_GIMBAL_ANGLE, MAX_GIMBAL_ANGLE);

        // Translate origin to our reference point
        read_table(TABLE_IDX_ZERO_REF_OUTER_GIMBAL, (uint8_t*)&zero_ref_outer, sizeof(float));
        read_table(TABLE_IDX_ZERO_REF_INNER_GIMBAL, (uint8_t*)&zero_ref_inner, sizeof(float));
        a_outer += zero_ref_outer;
        a_inner += zero_ref_inner;

        // Update motor angles
        lss_set_position(&servo_outer, a_outer);
        lss_set_position(&servo_inner, a_inner);
    }
  /* USER CODE END StartControlTask */
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
        if (huart->Instance == PC_UART.Instance && TxSemHandle != NULL){
            xSemaphoreGiveFromISR(TxSemHandle, &xHigherPriorityTaskWoken);
        }
        else if(huart->Instance == USART1){
            xTaskNotifyFromISR(ControlHandle, NOTIFIED_FROM_TX_ISR, eSetBits, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (scheduler_has_started)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (huart->Instance == PC_UART.Instance && RxSemHandle != NULL){
            xSemaphoreGiveFromISR(RxSemHandle, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
