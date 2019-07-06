/**
 * @file sensing.h
 * @author Tyler
 *
 * @defgroup Sensing
 * @brief Sensing acceleration and angular velocity
 * @{
 */

#ifndef SENSING_H_
#define SENSING_H_




/********************************** Includes *********************************/
#include <stdbool.h>
#include "MPU6050/MPU6050.h"



/******************************* Public variables ****************************/
extern MPU6050_t imu_base;
extern MPU6050_t imu_lamp;




/********************************* Functions *********************************/
bool init_imu(MPU6050_t* imu, I2C_HandleTypeDef* hi2c);

/**
 * @}
 */
/* end - Sensing */

#endif /* SENSING_H_ */
