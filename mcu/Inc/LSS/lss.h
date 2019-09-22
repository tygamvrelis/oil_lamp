/**
 * @file lss.h
 * @author Tyler
 *
 * @defgroup LSS
 * @{
 */




#ifndef LSS_H
#define LSS_H




/********************************* Includes **********************************/
#include <stdint.h>
#include "usart.h"




/****************************** Public variables ****************************/
extern const uint8_t BROADCAST_ID;




/********************************** Types ************************************/
typedef struct
{
    uint8_t id;                 /**< Motor identification */
    UART_HandleTypeDef* p_uart; /**< UART handle for motor */
}lss_t;




/***************************** Function prototypes ***************************/
/**
 * @brief Sends command to move the servo to the specified angle
 * @param hlss Handle for the motor
 * @param angle Desired angle for motor
 * TODO: Support timed move (T) and speed (S) modifiers
 */
void lss_set_position(lss_t* hlss, float angle);

/**
 * @brief Sets the motor's baud rate
 * @param hlss Handle for the motor
 * @param baud_rate Baud rate to set. Must be one of 9600, 19200, 38400, 57600,
 *        115200, 230400, 250000, 460800, 500000
 */
void lss_set_baud(lss_t* hlss, uint32_t baud_rate);

/**
 * @brief Sets the motor's ID. Motor must be reset for this to take effect
 * @param hlss Handle for the motor
 * @param id ID to program into the motor
 */
void lss_set_id(lss_t* hlss, uint8_t id);

/**
 * @brief Varies how much quickly the motor accelerates/decelerates to get to
 *        goal position
 * @param hlss Handle for the motor
 * @param angular_stiffness Value in [-10,10] determining how much torque is
 *        applied near goal position (>0 => more torque applied, <0 => less)
 */
void lss_set_as(lss_t* hlss, int8_t angular_stiffness);

/**
 * @brief Varies the servo's ability to hold a desired position under load
 * @param hlss Handle for the motor
 * @param holding_stiffness Value in [-10,10]. Increasing values become
 *        increasingly erratic
 */
void lss_set_hs(lss_t* hlss, int8_t holding_stiffness);

/**
 * @brief Varies the servo's angular acceleration
 * @param hlss Handle for the motor
 * @param angular_accel Angular acceleration in increments of 10 deg/s/s
 */
void lss_set_aa(lss_t* hlss, int8_t angular_accel);

/**
 * @brief Varies the servo's angular deceleration
 * @param hlss Handle for the motor
 * @param angular_decel Angular deceleration in increments of 10 deg/s/s
 */
void lss_set_ad(lss_t* hlss, int8_t angular_decel);

#endif /* LSS_H */
