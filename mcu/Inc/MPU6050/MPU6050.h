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
#include <stdbool.h>
#include "i2c.h"
#include "MPU6050_t.h"




/********************************* Functions *********************************/
/**
 * @brief  Initializes the MPU6050 sensor by writing various settings to its
 *         registers, and reading a few of them for verification
 * @note   Should only be used prior to starting the scheduler. Uses blocking
 *         IO
 * @param  imu Pointer to the data structure representing the MPU6050
 * @param  hi2c I2C instance the sensor is attached to
 * @return true if successful, otherwise false
 */
bool mpu6050_init(MPU6050_t* imu, I2C_HandleTypeDef* hi2c);

/**
 * @brief Reset the IMU
 * @note  Should only be used prior to starting the scheduler. Uses blocking
 *        IO
 * @param imu Pointer to the data structure representing the MPU6050
 */
void mpu6050_reset(MPU6050_t* imu);

/* @brief Associates a semaphore with a MPU6050 instance so that it can
 *        synchronize with interrupts
 * @param imu Pointer to the data structure representing the MPU6050
 * @param sem Handle for the semaphore to take while transferring data
 */
void mpu6050_attach_semaphore(MPU6050_t* imu, osSemaphoreId sem);

// These are the only ones that should be used during runtime once the
// scheduler has started
/**
 * @brief  Read from the az, ay, ax register addresses and stores the results in
 *         the IMU object passed in
 * @param  imu Pointer to the data structure representing the MPU6050
 * @return true if successful, otherwise false
 */
bool mpu6050_read_accel(MPU6050_t* imu);

/**
 * @brief  Read from the vz, vy, vx register addresses and stores the results in
 *         the IMU object passed in
 * @param  imu Pointer to the data structure representing the MPU6050
 * @return true if successful, otherwise false
 */
bool mpu6050_read_gyro(MPU6050_t* imu);

/**
 * @return Sensor data from the IMU
 */
imu_data_t mpu6050_get_data(MPU6050_t* imu);

/**
 * @brief This function bit-bangs the I2C master clock, with the option of
 *        sending stop bits
 * @note  Use with EXTREME caution
 */
void mpu6050_generate_clocks(
	I2C_HandleTypeDef* hi2c,
	uint8_t numClocks,
	uint8_t sendStopBits
);

/**
 * @}
 */
/* end - MPU6050_Driver */

#endif /* MPU6050_H_ */
