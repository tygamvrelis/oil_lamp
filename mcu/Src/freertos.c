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
#include "tim.h"
#include "wwdg.h"
#include "App/util.h"
#include "App/table.h"
#include "App/sensing.h"
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
osThreadId ImuHandle;
uint32_t ImuTaskBuffer[ 128 ];
osStaticThreadDef_t ImuTaskControlBlock;
osThreadId ControlHandle;
uint32_t ControlTaskBuffer[ 512 ];
osStaticThreadDef_t ControlTaskControlBlock;
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

/** @brief Enables the sensing (IMUs) thread */
void enable_sensing()
{
    xTaskNotify((TaskHandle_t)ImuHandle, NOTIFY_CLEAR, eSetValueWithOverwrite);
}

/** @brief Disables the sensing (IMUs) thread */
void disable_sensing()
{
    xTaskNotify((TaskHandle_t)ImuHandle, NOTIFY_THREAD_DI, eSetBits);
}

/** @brief Returns true if the current thread is enabled, otherwise false */
bool thread_is_enabled(){
    uint32_t ulNotifiedValue;
    // Don't clear bits and don't block
    xTaskNotifyWait(0, 0, &ulNotifiedValue, 0);
    return (ulNotifiedValue & NOTIFY_THREAD_DI) != NOTIFY_THREAD_DI;
}
/* USER CODE END FunctionPrototypes */

void StartRxTask(void const * argument);
void StartTxTask(void const * argument);
void StartImuTask(void const * argument);
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

  /* definition and creation of Imu */
  osThreadStaticDef(Imu, StartImuTask, osPriorityNormal, 0, 128, ImuTaskBuffer, &ImuTaskControlBlock);
  ImuHandle = osThreadCreate(osThread(Imu), NULL);

  /* definition and creation of Control */
  osThreadStaticDef(Control, StartControlTask, osPriorityNormal, 0, 512, ControlTaskBuffer, &ControlTaskControlBlock);
  ControlHandle = osThreadCreate(osThread(Control), NULL);

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
    static const uint32_t RX_CYCLE_TIME = osKernelSysTickMicroSec(1000);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    static uint8_t rx_buff[32] = {0}; // static => no stack usage
    CircBuff_t circ_buff = {sizeof(rx_buff), 0, 0, rx_buff};
    float tmp_angle = 0.0;
    uint8_t cnt = 0;

    enum {
        CMD_NONE,
        CMD_BLINK = 'L',
        CMD_CTRL_DI = '0',
        CMD_CTRL_EN = '1',
        CMD_SENS_DI = '2',
        CMD_SENS_EN = '3',
        CMD_ANGLE = 'A',
        CMD_ANGLE_OUTER,
        CMD_ANGLE_INNER
    } parse_state = CMD_NONE;

    MX_WWDG_Init();
    disable_control();
    HAL_UART_Receive_DMA(&huart2, rx_buff, sizeof(rx_buff));
    for(;;)
    {
        osDelayUntil(&xLastWakeTime, RX_CYCLE_TIME);
        circ_buff.iHead = circ_buff.size - huart2.hdmarx->Instance->NDTR;
        while(circ_buff.iHead != circ_buff.iTail)
        {
            uint8_t data = pop(&circ_buff);
            switch (parse_state)
            {
                case CMD_NONE:
                    if ((char)data == (char)CMD_BLINK)
                    {
                        parse_state = CMD_BLINK;
                    }
                    else if ((char)data == (char)CMD_ANGLE)
                    {
                        parse_state = CMD_ANGLE;
                        cnt = 0;
                    }
                    else if ((char)data == (char)CMD_CTRL_DI)
                    {
                        disable_control();
                    }
                    else if ((char)data == (char)CMD_CTRL_EN)
                    {
                        enable_control();
                    }
                    else if ((char)data == (char)CMD_SENS_DI)
                    {
                        disable_sensing();
                    }
                    else if ((char)data == (char)CMD_SENS_EN)
                    {
                        enable_sensing();
                    }
                    break;
                default:
                    break;
            }

            // Take action based on state
            switch (parse_state)
            {
                case CMD_BLINK:
                    blink_camera_led();
                    parse_state = CMD_NONE;
                    break;
                case CMD_ANGLE:
                    parse_state = CMD_ANGLE_OUTER;
                    break;
                case CMD_ANGLE_OUTER:
                    ((uint8_t*)&tmp_angle)[cnt] = data;
                    ++cnt;
                    if (cnt == sizeof(float))
                    {
                        write_table(
                            TABLE_IDX_OUTER_GIMBAL_ANGLE,
                            (uint8_t*)&tmp_angle,
                            sizeof(float)
                        );
                        parse_state = CMD_ANGLE_INNER;
                        cnt = 0;
                    }
                    break;
                case CMD_ANGLE_INNER:
                    ((uint8_t*)&tmp_angle)[cnt] = data;
                    ++cnt;
                    if (cnt == sizeof(float))
                    {
                        write_table(
                            TABLE_IDX_INNER_GIMBAL_ANGLE,
                            (uint8_t*)&tmp_angle,
                            sizeof(float)
                        );
                        parse_state = CMD_NONE;
                        cnt = 0;
                    }
                    break;
                default:
                    break;
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

/* USER CODE BEGIN Header_StartImuTask */
/**
* @brief Function implementing the Imu thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartImuTask */
void StartImuTask(void const * argument)
{
  /* USER CODE BEGIN StartImuTask */
    while (!thread_is_enabled())
    {
        continue;
    }

    static const uint32_t IMU_CYCLE_TIME = osKernelSysTickMicroSec(IMU_CYCLE_MS * 1000);

    imu_data_t imu_data;
    mpu6050_attach_semaphore(&imu_lamp, LampSemHandle);
    mpu6050_attach_semaphore(&imu_base, BaseSemHandle);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    osDelay(TX_PERIOD_MS - 2);
    for(;;)
    {
        osDelayUntil(&xLastWakeTime, IMU_CYCLE_TIME);
        if (!thread_is_enabled())
        {
            continue;
        }

        mpu6050_read_accel(&imu_lamp);
        mpu6050_read_gyro(&imu_lamp);
        imu_data = mpu6050_get_data(&imu_lamp);
        write_table(TABLE_IDX_LAMP_DATA, (uint8_t*)&imu_data, sizeof(imu_data_t));

        mpu6050_read_accel(&imu_base);
        mpu6050_read_gyro(&imu_base);
        imu_data = mpu6050_get_data(&imu_base);
        write_table(TABLE_IDX_BASE_DATA, (uint8_t*)&imu_data, sizeof(imu_data_t));
    }
  /* USER CODE END StartImuTask */
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
    float a_outer, a_inner;

    lss_set_io_type(IO_DMA);

    // All observations below are for a no-load scenario
    // => AS=0, HS=-10, Speed=180.0, AA=15, AD=15 -- tracks well when I
    // tell it to go from point A to point B (no overshoot), but doesn't like waveforms
    //
    // => AS=0, HS=0, Speed=180.0, AA=15, AD=15 -- slight jump right before reaching
    // goal position. Sine waves stop very suddenly near goal position; not nice to look at
    //
    // => AS=0, HS=10, Speed=180.0, AA=15, AD=15 -- overshoots goal position slightly. Not
    // nice for waveforms
    //
    // => AS=10, HS=10, Speed=180.0, AA=15, AD=15 -- Jittery whole way through
    //
    // => AS=-10, HS=10, Speed=180.0, AA=15, AD=15 -- Slows down near goal position, but has
    // a huge jump at the end
    //
    // => AS=-3, HS=10, Speed=180.0, AA=15, AD=15 -- Slows down near goal position, but has
    // a jump at the end (not as large as when AS=-10 though)
    //
    // => AS=5,0,-5, HS=8, Speed=180.0, AA=15, AD=15 -- Fairly similar to the above couple trials,
    // no noticeable improvement
    //
    // => AS=-10, HS=8,6, Speed=180.0, AA=15, AD=15 -- Big jump near goal position again
    //
    // => AS=-5,0,5, HS=6, Speed=180.0, AA=15, AD=15 -- smaller yet noticeable jump near goal pos
    //
    // => AS=5,-5, HS=4, Speed=180.0, AA=15, AD=15 -- still jumping
    //
    // => AS=0, HS=4, Speed=180.0, AA=15, AD=15 -- still jumping, but maybe a bit smoother than before
    //
    // => AS=-5,0, HS=2, Speed=180.0, AA=15, AD=15 -- jumping tiny bit near goal
    //
    // => AS=5, HS=2, Speed=180.0, AA=15, AD=15 -- larger jump near goal
    //
    // => AS=0, HS=4, Speed=180.0, AA=15, AD=100 -- No perceptible jump near goal position. Not great
    // waveform tracking (still jumps around)
    //
    // => AS=0, HS=4, Speed=180.0, AA=100, AD=100 -- No perceptible jump near goal position. Great
    // tracking of 0.1 Hz sine wave amplitude 20. Jumps for 0.1 Hz amplitude 40 and 0.2 Hz amplitude 20
    //
    // => AS=0, HS=4, Speed=180.0, AA=10, AD=100 -- No perceptible jump near goal position. Great
    // tracking of 0.1 Hz sine wave amplitude 20. Jumps for 0.1 Hz amplitude 30 and 0.2 Hz amplitude 20
    //
    // => AS=0, HS=4, Speed=180.0, AA=100, AD=10 -- Slight jump near goal position. Great
    // tracking of 0.1 Hz sine wave amplitude 20. Tracks 0.2 Hz amplitude 20-40 with occasional small
    // jumps, but far better than any of the other settings tried so far. Small jumps tended to occur
    // most often near the extrema of the periodic motion (when displacement was max and velocity was min).
    // Also seems to be handling 0.3 Hz amplitude 30 reasonably well, again with small hiccups near the
    // extrema. 0.3 Hz amplitude 40 was noticeably more jittery than 0.3 Hz amplitude 30
    //
    // => AS=0, HS=4, Speed=180.0, AA=100, AD=1 -- Better tracking of 0.3 Hz amplitude 40 wave. Doesn't quite
    // handle 0.4 Hz amplitude 40
    //
    // => AS=0, HS=4, Speed=10.0, AA=100, AD=1 -- Tracking seems ok up to 0.3 Hz amplitude 40 wave. If
    // we send 0.4 Hz amplitude 40, it starts to jump through the middle section (max velocity part) of
    // the trajectory, suggesting that capping the speed isn't a good idea
    //
    // => AS=3, HS=4, Speed=180.0, AA=100, AD=1 -- re: 0.4 Hz ampl 40, it has a more "discrete" feeling
    // throughout, but motion feels more consistent
    //
    // => AS=6, HS=4, Speed=180.0, AA=100, AD=1 -- re: 0.4 Hz ampl 40, noticeably more coarse, but speed
    // feels right
    //
    // => AS=9, HS=4, Speed=180.0, AA=100, AD=1 -- re: 0.4 Hz ampl 40, feels very coarse, and a bit jittery.
    // At 0.5 Hz ampl 40, jittery is very noticeably. Tracks perfectly fine at 0.1 Hz ampl 40. Moves from
    // point A to point B very slowly
    //
    // => AS=-3,-1, HS=4, Speed=180.0, AA=100, AD=1 -- re: 0.4 Hz ampl 40, feels smooth, but motor does this
    // weird thing where it accelerates HARD right to the final angle. So it really doesn't track the sine,
    // but rather, it goes to one extremum, then accelerates all the way into the other one, and so on. All
    // in all, AS < 0 doesn't seem to be good
    //
    // => AS=-10,4, HS=4, Speed=10.0, AA=10, AD=1 -- 0.5 Hz ampl 40 sine wave is handled w/o being too erratic,
    // but stops abruptly near extrema. 1 Hz sine wave at ampl 40 probably doesn't go beyond +/- 5 degrees in
    // reality at this setting
    //
    // COMMENT: So far AS=0, HS=4, Speed=180.0, AA=100, AD=1 seems like the best setting for slow motions. For
    // fast, I'm still not sure
    //
    // COMMENT:
    // => AS=0, HS=0, Speed=180.0, AA=100, AD=100 => goes from point A to point B quickly
    // => AS=0, HS=0, Speed=180.0, AA=100, AD=1 => goes from point A to point B very slowly
    // TAKEAWAY: AD > 1 should be necessary for fast motion
    //
    // => AS=0, HS=0, Speed=180.0, AA=100, AD=100 => jumpy waveform tracking, caused servo to exceed one of its limits
    //
    // => AS=0, HS=0, Speed=180.0, AA=100, AD=50 => jumpy at 1 Hz ampl 40, although less than the above
    //
    // => AS=0, HS=4, Speed=180.0, AA=100, AD=50 => Jittery at 1 Hz
    //
    // => AS=0, HS=-4, Speed=180.0, AA=100, AD=50 => Jittery at 1 Hz, although less than the above
    //
    // => AS=0, HS=0, Speed=180.0, AA=100, AD=75 => Similar to the above
    //
    // => AS=0, HS=0, Speed=10.0, AA=100, AD=75 => Smooth at 1 Hz, but doesn't exhibit noticeable change in
    // velocity except when turning around at extrema
    //
    // => AS=0, HS=0, Speed=20.0, AA=100, AD=75 => Smooth at 1 Hz, exhibits better speed characteristic than
    // the above. Smooth at 0.1 Hz ampl 40, but jumpy at 0.3 Hz ampl 40
    //
    // => AS=1, HS=0,1, Speed=20.0, AA=100, AD=75 => fairly similar, both are not great at extrema
    //
    // => AS=0, HS=0, Speed=20.0, AA=100, AD=100 => Pretty nice tracking at 1 Hz ampl 20. Hits speed limit on
    // 1 Hz ampl 40 though, it seems, and not great at extrema for ampl 40
    // COMMENT: Above setting is the best I've seen so far for fast oscillations
    //
    // => AS=0, HS=0, Speed=25.0, AA=100, AD=100 => More jumpy than the above for 1 Hz ampl 20
    //
    // => AS=0, HS=4, Speed=20.0, AA=100, AD=100 => Seems more coarse at 1 Hz. Jumpy above 0.1 Hz
    const int8_t ANGULAR_STIFFNESS = -4;
    const int8_t HOLDING_STIFFNESS = 4;
    const uint8_t OUTER_ID = 1;
    lss_t servo_outer = {OUTER_ID, &huart1};
    lss_set_speed(&servo_outer, 180.0);
    lss_set_aa(&servo_outer, 100);
    lss_set_ad(&servo_outer, 100);
    lss_set_as(&servo_outer, ANGULAR_STIFFNESS);
    lss_set_hs(&servo_outer, HOLDING_STIFFNESS);

    const uint8_t INNER_ID = 0;
    lss_t servo_inner = {INNER_ID, &huart1};
    //lss_set_as(&servo_inner, ANGULAR_STIFFNESS);
    //lss_set_as(&servo_inner, HOLDING_STIFFNESS);

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
void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg)
{
    // Watchdog wants to reset
    // Check if I2C is frozen, if so, try to software-reset it. Otherwise, let
    // the reset occur.
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
        HAL_WWDG_Refresh(hwwdg);
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
