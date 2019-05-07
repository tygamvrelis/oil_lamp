/**
 * @file MPU6050_t.h
 * @author Tyler
 *
 * @ingroup MPU6050_Driver
 */

#ifndef MPU6050_T_H_
#define MPU6050_T_H_




/********************************** Includes *********************************/
#include "cmsis_os.h"




/*********************************** Types ************************************/
/**
 * @brief The data structure which stores the data read from the MPU6050 sensor
 */
typedef struct{
	I2C_HandleTypeDef* hi2c; /**< I2C for this instance  */
	osSemaphoreId sem; /**< Semaphore for async IO       */
    float az; /**< Acceleration along z-axis             */
    float ay; /**< Acceleration along y-axis             */
    float ax; /**< Acceleration along x-axis             */
    float vz; /**< Yaw rate (about z-axis)               */
    float vy; /**< Pitch rate (about y-axis)             */
    float vx; /**< Roll rate (about x-axis)              */
}MPU6050_t;

#endif /* MPU6050_T_H_ */
