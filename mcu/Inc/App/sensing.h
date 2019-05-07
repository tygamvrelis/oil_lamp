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




/*********************************** Types ************************************/
/** @brief Holds state and parameterization for a complementary filter */
typedef struct
{
	float last_theta; /**< Last angle, for feedback */
	float alpha; /**< Velocity weight, equal to 1 minus acceleration weight */
}cFilt_t;



/******************************* Public variables ****************************/
extern MPU6050_t imu_base;
extern MPU6050_t imu_lamp;




/********************************* Functions *********************************/
bool init_imu(MPU6050_t* imu, I2C_HandleTypeDef* hi2c);

/**
 * @brief Estimates position from angular velocity and acceleration according
 *        to the update rule:
 *        theta(t) = alpha*(theta(t-1) + v1(t)) + (alpha - 1)*arctan2(-a2/a3)
 *        In an orthogonal coordinate system labeled by 1, 2, 3.
 * @example v1 = vy, a2 = ax, a3 = az estimates the angle made with the x-y plane
 * @param filt Filter instance
 * @param v1 Angular velocity about the axis of rotation
 * @param a2 Acceleration in the 1-2 plane, along coordinate 2
 * @param a3 Acceleration along the 3rd coordinate
 * @param dt Time elapsed between updates, in milliseconds
 */
float cFilt_update(cFilt_t* filt, float v1, float a2, float a3, float dt);

/**
 * @}
 */
/* end - Sensing */

#endif /* SENSING_H_ */
