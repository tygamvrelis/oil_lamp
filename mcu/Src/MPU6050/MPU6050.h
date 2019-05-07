/**
 * @file MPU6050.h
 * @author Tyler
 *
 * @defgroup MPU6050_Driver
 * @brief Library for interfacing with MPU6050
 * @{
 */

#ifndef MPU6050_H_
#define MPU6050_H_




/********************************** Includes *********************************/
#include <math.h>
#include "cmsis_os.h"
#include "i2c.h"
#include "MPU6050_t.h"




/********************************* Functions *********************************/
// These should only be used before the scheduler is started
int MPU6050Init(MPU6050_t* myMPU, I2C_HandleTypeDef* hi2c);
void resetIMUBlocking(MPU6050_t* myMPU);

// These are the only ones that should be used during runtime once the
// scheduler has started
/**
 * @brief  Read from the az, ay, ax register addresses and stores the results in
 *         the myMPU object passed in
 * @param  myMPU Pointer to the data structure which stores the data read from
 *         the MPU6050 sensor
 * @param  sem Handle for the semaphore to take while transferring data
 * @return 1 if successful, -1 otherwise
 */
int accelReadDMA(MPU6050_t* myMPU, osSemaphoreId sem);

/**
 * @brief  Read from the az, ay, ax register addresses and stores the results in
 *         the myMPU object passed in
 * @param  myMPU Pointer to the data structure which stores the data read from
 *         the MPU6050 sensor
 * @param  sem Handle for the semaphore to take while transferring data
 * @return 1 if successful, -1 otherwise
 */
int gyroReadDMA(MPU6050_t* myMPU, osSemaphoreId sem);

/**
 * @}
 */
/* end - MPU6050_Driver */

#endif /* MPU6050_H_ */
