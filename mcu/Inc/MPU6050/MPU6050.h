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
#include "i2c.h"
#include "MPU6050_t.h"




/********************************* Functions *********************************/
// These should only be used before the scheduler is started
int MPU6050Init(MPU6050_t* myMPU, I2C_HandleTypeDef* hi2c);
void resetIMUBlocking(MPU6050_t* myMPU);

/* @brief Associates a semaphore with this MPU6050 instance so that it can
 *        synchronize with interrupts
 * @param myMPU Pointer to the data structure which stores the data read from
 *        the MPU6050 sensor
 * @param sem Handle for the semaphore to take while transferring data
 */
void attachSemaphore(MPU6050_t* myMPU, osSemaphoreId sem);

// These are the only ones that should be used during runtime once the
// scheduler has started
/**
 * @brief  Read from the az, ay, ax register addresses and stores the results in
 *         the myMPU object passed in
 * @param  myMPU Pointer to the data structure which stores the data read from
 *         the MPU6050 sensor
 * @return 1 if successful, -1 otherwise
 */
int accelReadIT(MPU6050_t* myMPU);

/**
 * @brief  Read from the az, ay, ax register addresses and stores the results in
 *         the myMPU object passed in
 * @param  myMPU Pointer to the data structure which stores the data read from
 *         the MPU6050 sensor
 * @return 1 if successful, -1 otherwise
 */
int gyroReadIT(MPU6050_t* myMPU);

imu_data_t get_data(MPU6050_t* myMPU);

void generateClocks(
	I2C_HandleTypeDef* hi2c,
	uint8_t numClocks,
	uint8_t sendStopBits
); // Use with extreme caution

/**
 * @}
 */
/* end - MPU6050_Driver */

#endif /* MPU6050_H_ */
