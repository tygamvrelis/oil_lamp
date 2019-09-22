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

/** @brief Enumerates the low-level I/O modes the library supports */
typedef enum IO_FLAGS{
    IO_DMA,  /**< Direct memory access */
    IO_POLL, /**< Polled I/O           */
    IO_IT    /**< Interrupt-based I/O  */
}ioFlags_t;




/***************************** Function prototypes ***************************/
/**
 * @brief   Sets the I/O type used by the library
 * @details Sets the IO protocol to one of three options:
 *              -# Blocking (Polling)
 *              -# Non-Blocking (Interrupt)
 *              -# DMA
 * @param   type one of IO_POLL, IO_IT, or IO_DMA
 * @return  None
 */
void lss_set_io_type(enum IO_FLAGS type);

/**
 * @brief   Gets the IO protocol setting for the library
 * @return  One of IO_POLL, IO_IT, or IO_DMA
 */
enum IO_FLAGS lss_get_io_type();

/**
 * @brief Soft reset of motor
 * @param hlss Handle for the motor
 */
void lss_reset(lss_t* hlss);

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
 * @param angular_accel Angular acceleration in increments of 10 deg/s/s in
 *        range [1,100]
 */
void lss_set_aa(lss_t* hlss, uint8_t angular_accel);

/**
 * @brief Varies the servo's angular deceleration
 * @param hlss Handle for the motor
 * @param angular_decel Angular deceleration in increments of 10 deg/s/s in
 *        range [1,100]
 */
void lss_set_ad(lss_t* hlss, uint8_t angular_decel);

#endif /* LSS_H */
