/**
 * @file MPU9250_t.h
 * @author Tyler
 *
 * @ingroup MPU9250_Driver
 */

#ifndef MPU9250_MPU9250_T_H_
#define MPU9250_MPU9250_T_H_




/*********************************** Types ************************************/
/**
 * @brief The data structure which stores the data read from the MPU9250 sensor
 */
typedef struct{
    float az; /**< Acceleration along z-axis             */
    float ay; /**< Acceleration along y-axis             */
    float ax; /**< Acceleration along x-axis             */
    float A;  /**< ||a||                                 */
    float vz; /**< Yaw rate (about z-axis)  (DEPRECATED) */
    float vy; /**< Pitch rate (about y-axis)(DEPRECATED) */
    float vx; /**< Roll rate (about x-axis) (DEPRECATED) */
    float hx; /**< Magnetic field along x                */
    float hy; /**< Magnetic field along y                */
    float hz; /**< Magnetic field along z                */
}MPU9250_t;

#endif /* MPU9250_MPU9250_T_H_ */
