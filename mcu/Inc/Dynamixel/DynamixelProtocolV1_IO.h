/**
 * @file DynamixelProtocolV1_IO.h
 * @author Tyler
 *
 * @defgroup DynamixelProtocolV1_IO_Header Header
 * @ingroup DynamixelProtocolV1_IO
 * @{
 */




#ifndef DYNAMIXEL_H_DYNAMIXELPROTOCOLV1_IO_H_
#define DYNAMIXEL_H_DYNAMIXELPROTOCOLV1_IO_H_

#ifdef __cplusplus
extern "C" {
#endif




/********************************* Includes **********************************/
#define THREADED
#include <stdint.h>
#include <stdbool.h>
#include "Dynamixel_Types.h"




/***************************** Function prototypes ***************************/
void Dynamixel_SetIOType(enum IO_FLAGS type);

enum IO_FLAGS Dynamixel_GetIOType();

// Low-level transmission and reception functions
void Dynamixel_DataWriter(
        Dynamixel_HandleTypeDef* hdynamixel,
        uint8_t* args,
        uint8_t numArgs
);

uint16_t Dynamixel_DataReader(
        Dynamixel_HandleTypeDef* hdynamixel,
        uint8_t readAddr,
        uint8_t readLength
);

/**
 * @}
 */
/* end - DynamixelProtocolV1_IO_Header */

#ifdef __cplusplus
}
#endif

#endif /* DYNAMIXEL_H_DYNAMIXELPROTOCOLV1_IO_H_ */
