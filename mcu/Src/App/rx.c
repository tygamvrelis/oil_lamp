/**
 * @file rx.c
 * @author Tyler
 *
 * @ingroup RX
 * @{
 */




/********************************** Includes *********************************/
#include "App/rx.h"




/***************************** Public Functions ******************************/
uint8_t pop(CircBuff_t* buff)
{
    uint8_t data = buff->pBuff[buff->iTail];
    ++buff->iTail;
    if (buff->iTail == buff->size)
    {
        buff->iTail = 0;
    }
    return data;
}

/**
 * @}
 */
/* end - RX */
