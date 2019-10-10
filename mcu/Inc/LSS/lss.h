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
#include <stdbool.h>
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

typedef enum COLORS{
    LSS_OFF = 0,
    LSS_BLACK = 0,
    LSS_RED,
    LSS_GREEN,
    LSS_BLUE,
    LSS_YELLOW,
    LSS_CYAN,
    LSS_MAGENTA,
    LSS_WHITE,
    LSS_COLOR_MAX
}lss_colors_t;

typedef enum EM
{
    LSS_EM0=0, // Disable motion control
    LSS_EM1=1, // Enable motion control
    LSS_EM_MAX
}lss_em_t;




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
 * @param hlss Handle for the motor in degrees
 * @param angle Desired angle for motor
 */
void lss_set_position(lss_t* hlss, float angle);

/**
 * @brief Sends position command with timed move (T) modifier
 * @param hlss Handle for the motor
 * @param angle Desired angle for motor in degrees
 * @param time_ms Desired travel time to goal position, expresse din ms
 */
void lss_set_position_timed(lss_t* hlss, float angle, uint16_t time_ms);

/**
 * @brief Sets the servo's max speed
 * @param hlss Handle for the motor
 * @param deg_per_sec Max speed in degrees/s. Up to 180.0 is allowed
 */
void lss_set_speed(lss_t* hlss, float deg_per_sec);

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
 * @brief Sets the LED color
 * @param hlss Handle for the motor
 * @param color LED color
 */
void lss_set_led(lss_t* hlss, lss_colors_t color);

/**
 * @brief Turns the motion control algorithms in the servo on or off
 * @param enable Enables motion control if true, else disables it
 */
void lss_toggle_motion_ctrl(lss_t* hlss, lss_em_t enable);

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
void lss_set_aa(lss_t* hlss, uint16_t angular_accel);

/**
 * @brief Varies the servo's angular deceleration
 * @param hlss Handle for the motor
 * @param angular_decel Angular deceleration in increments of 10 deg/s/s in
 *        range [1,100]
 */
void lss_set_ad(lss_t* hlss, uint16_t angular_decel);

#endif /* LSS_H */
