/**
 * @file servo.h
 * @author Tyler
 *
 * @defgroup Servo
 * @brief Servo control
 * @{
 */

#ifndef SERVO_H_
#define SERVO_H_




/********************************** Includes *********************************/
#include <stdint.h>
#include "tim.h"




/****************************** Public variables ****************************/
extern const int8_t POS_MAX;
extern const int8_t POS_MIN;




/*********************************** Types ************************************/
/** Enumerates the servos in the system */
typedef enum
{
    SERVO_OUTER,
    SERVO_INNER
}Servo_e;

/** Aggregates data for interfacing with servo */
typedef struct
{
    Servo_e id;              /**< Unique identifier for the servo */
    TIM_HandleTypeDef* htim; /**< Timer associated with this servo */
    uint32_t channel;        /**< PWM channel associated with this servo */
}Servo_t;




/********************************* Functions *********************************/
/**
 * @brief Initializes the servo instance and starts the signal timer
 * @param p_servo Pointer to servo instance
 * @param id Identifier to assign to servo
 * @param htim Timer instance to assign to servo
 * @param channel Channel to assign to servo
 */
void servo_init(
    Servo_t* p_servo,
    Servo_e id,
    TIM_HandleTypeDef* htim,
    uint32_t channel
);

/**
 * @brief Changes the servo's position by updating the PWM duty cycle
 * @param p_servo Pointer to servo instance
 * @param angle The angle to move the servo to
 */
void servo_set_position(Servo_t* p_servo, int8_t angle);

/**
 * @}
 */
/* end - Servo */

#endif /* SERVO_H_ */
