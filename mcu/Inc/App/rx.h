/**
 * @file rx.h
 * @author Tyler
 *
 * @defgroup RX
 * @brief Reception
 * @{
 */

#ifndef RX_H_
#define RX_H_




/********************************** Includes *********************************/
#include <stdint.h>




/*********************************** Types ************************************/
typedef struct
{
    const uint8_t size;
    uint8_t iHead;
    uint8_t iTail;
    uint8_t* pBuff;
}CircBuff_t;




/********************************* Functions *********************************/
uint8_t pop(CircBuff_t* buff);

/**
 * @}
 */
/* end - RX */

#endif /* RX_H_ */
