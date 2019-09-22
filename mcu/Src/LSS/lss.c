/**
 * @file Servo.c
 * @author Tyler
 *
 * @ingroup LSS
 * @{
 */




/********************************** Includes *********************************/
#include "LSS/lss.h"
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#define THREADED

#if defined(THREADED)
#include "App/Notification.h"
#include "cmsis_os.h"
#endif




/****************************** Public variables ****************************/
const uint8_t BROADCAST_ID = 254;




/****************************** Private variables ****************************/
static const uint32_t TRANSMIT_TIMEOUT = 1;

/**
 * @brief Configures the low-level I/O mode used by the library
 *        Default: polled I/O
 */
ioFlags_t IOType = IO_POLL;




/****************************** Private Functions ****************************/
bool lss_transmit(lss_t* hlss, uint8_t* arr, uint8_t length)
{
#if defined(THREADED)
    uint32_t notification;
    BaseType_t status;
#endif
    bool retval = true;

    switch(IOType) {
#if defined(THREADED)
        case IO_DMA:
            HAL_UART_Transmit_DMA(hlss->p_uart, arr, length);

            status = xTaskNotifyWait(0, NOTIFIED_FROM_TX_ISR, &notification, MAX_DELAY_TIME);

            if(status != pdTRUE || !CHECK_NOTIFICATION(notification, NOTIFIED_FROM_TX_ISR)){
                retval = false;
            }
            break;
        case IO_IT:
            HAL_UART_Transmit_IT(hlss->p_uart, arr, length);

            status = xTaskNotifyWait(0, NOTIFIED_FROM_TX_ISR, &notification, MAX_DELAY_TIME);

            if(status != pdTRUE || !CHECK_NOTIFICATION(notification, NOTIFIED_FROM_TX_ISR)){
                retval = false;
            }
            break;
#endif
        case IO_POLL:
        default:
            HAL_UART_Transmit(hlss->p_uart, arr, length, TRANSMIT_TIMEOUT);
            break;
    }

    if(!retval){
        HAL_UART_AbortTransmit(hlss->p_uart);
    }

    return retval;
}



/***************************** Public Functions ******************************/
void lss_set_io_type(enum IO_FLAGS type) {
    IOType = type;
}

//-----------------------------------------------------------------------------

enum IO_FLAGS lss_get_io_type(){
    return IOType;
}

//-----------------------------------------------------------------------------

void lss_reset(lss_t* hlss)
{
    uint8_t buff[16];
    uint8_t length = snprintf((char*)buff, 16, "#%dRESET", hlss->id);
    buff[length] = '\r'; // Overwrite null character with cmd termination
    lss_transmit(hlss, buff, length + 1);
}

//-----------------------------------------------------------------------------

void lss_set_position(lss_t* hlss, float angle)
{
    uint8_t buff[16];
    uint8_t length = snprintf((char*)buff, 16, "#%dD%d", hlss->id, (int)(10.0 * angle));
    buff[length] = '\r'; // Overwrite null character with cmd termination
    lss_transmit(hlss, buff, length + 1);
}

//-----------------------------------------------------------------------------

void lss_set_baud(lss_t* hlss, uint32_t baud_rate)
{
    uint8_t buff[16];
    uint8_t length = snprintf((char*)buff, 16, "#%dCB%lu", hlss->id, baud_rate);
    buff[length] = '\r'; // Overwrite null character with cmd termination
    lss_transmit(hlss, buff, length + 1);
}

//-----------------------------------------------------------------------------

void lss_set_id(lss_t* hlss, uint8_t id)
{
    uint8_t buff[16];
    uint8_t length = snprintf((char*)buff, 16, "#%dCID%d", hlss->id, id);
    buff[length] = '\r'; // Overwrite null character with cmd termination
    lss_transmit(hlss, buff, length + 1);
    hlss->id = id;
}

//-----------------------------------------------------------------------------

void lss_set_led(lss_t* hlss, lss_colors_t color)
{
    if (color >= LSS_COLOR_MAX)
    {
        return;
    }
    uint8_t buff[16];
    uint8_t length = snprintf((char*)buff, 16, "#%dLED%d", hlss->id, color);
    buff[length] = '\r'; // Overwrite null character with cmd termination
    lss_transmit(hlss, buff, length + 1);
}

//-----------------------------------------------------------------------------

void lss_set_as(lss_t* hlss, int8_t angular_stiffness)
{
    uint8_t buff[16];
    uint8_t length = snprintf((char*)buff, 16, "#%dCAS%d", hlss->id, angular_stiffness);
    buff[length] = '\r'; // Overwrite null character with cmd termination
    lss_transmit(hlss, buff, length + 1);
}

//-----------------------------------------------------------------------------

void lss_set_hs(lss_t* hlss, int8_t holding_stiffness)
{
    uint8_t buff[16];
    uint8_t length = snprintf((char*)buff, 16, "#%dCA%d", hlss->id, holding_stiffness);
    buff[length] = '\r'; // Overwrite null character with cmd termination
    lss_transmit(hlss, buff, length + 1);
}

//-----------------------------------------------------------------------------

void lss_set_aa(lss_t* hlss, uint8_t angular_accel)
{
    uint8_t buff[16];
    uint8_t length = snprintf((char*)buff, 16, "#%dCA%d", hlss->id, angular_accel);
    buff[length] = '\r'; // Overwrite null character with cmd termination
    lss_transmit(hlss, buff, length + 1);
}

//-----------------------------------------------------------------------------


void lss_set_ad(lss_t* hlss, uint8_t angular_decel)
{
    uint8_t buff[16];
    uint8_t length = snprintf((char*)buff, 16, "#%dCD%d", hlss->id, angular_decel);
    buff[length] = '\r'; // Overwrite null character with cmd termination
    lss_transmit(hlss, buff, length + 1);
}

/**
 * @}
 */
/* end - LSS */
