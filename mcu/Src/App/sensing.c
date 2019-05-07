/**
 * @file sensing.c
 * @author Tyler
 * @brief Application code utilizing the MPU6050
 *
 * @ingroup Sensing
 * @{
 */




/********************************** Includes *********************************/
#include <math.h>
#include "App/sensing.h"




/******************************* Public variables ****************************/
MPU6050_t imu_base;
MPU6050_t imu_lamp;




/***************************** Public Functions ******************************/
bool init_imu(MPU6050_t* imu, I2C_HandleTypeDef* hi2c)
{
	// Attempt to initialize the MPU6050, up to 5 attempts
	int status;
	for(int i = 0; i < 5; i++){
		status = MPU6050Init(imu, hi2c);
	    if(status == 1){
	    	break;
	    }
	    else{
	    	// Try hard-resetting the IMU module
	        resetIMUBlocking(imu);
	    }
	    if(i == 2){
			/* When the microcontroller program starts up, it is not guaranteed that
			 * it is from a power cycle. Instead, the program may be starting up from
			 * a reset. A reset, however, only affects the microcontroller, and does not
			 * affect any peripherals connected to it. Thus, it is possible for the I2C
			 * bus to get locked when the slave is asserting an ACK and then the master
			 * clock drops out. The solution is to send some clock pulses to transition the
			 * state of the slave that's freezing the bus. */
			generateClocks(imu->hi2c, 10, 0); // Generate 10 clock periods
	    }
	    if(i == 3){
			/* There is a silicon bug in some ST MCUs where a filter in the I2C module
			 * randomly asserts the busy flag. The function below follows certain procedures
			 * presented in an errata document to alleviate the issue. */
	    	generateClocks(imu->hi2c, 10, 1); // Generate 10 clock periods
	    }
	    HAL_Delay(10);
	}
	return status == 1;
}

float cFilt_update(cFilt_t* filt, float v1, float a2, float a3, float dt)
{
	float a_term = (1.0 - filt->alpha) * atan2f(a2, a3) * 180.0 / M_PI;
	float v_term = filt->alpha * (filt->last_theta + v1 * dt / 1000.0);
	filt->last_theta = v_term + a_term;
	return filt->last_theta;
}

/**
 * @}
 */
/* end - Sensing */
