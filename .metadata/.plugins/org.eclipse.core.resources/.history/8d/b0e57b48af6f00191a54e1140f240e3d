/**
 * @file MPUFilter.h
 * @author Tyler
 *
 * @defgroup MPU9250_FIR_Header Header
 * @ingroup MPU9250_FIR
 * @{
 */

/******************************* SOURCE LICENSE *********************************
Copyright (c) 2018 MicroModeler.

A non-exclusive, nontransferable, perpetual, royalty-free license is granted to the Licensee to
use the following Information for academic, non-profit, or government-sponsored research purposes.
Use of the following Information under this License is restricted to NON-COMMERCIAL PURPOSES ONLY.
Commercial use of the following Information requires a separately executed written license agreement.

This Information is distributed WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

******************************* END OF LICENSE *********************************/

// A commercial license for MicroModeler DSP can be obtained at http://www.micromodeler.com/launch.jsp

// Link with library: libarm_cortexM4_mathL.a (or equivalent)
// Add CMSIS/Lib/GCC to the library search path
// Add CMSIS/Include to the include search path

#ifndef MPUFILTER_H_ // Include guards
#define MPUFILTER_H_




/********************************** Includes *********************************/
#define ARM_MATH_CM4	 // Use ARM Cortex M4
#include "stm32f446xx.h" // Need to know if we can generate FPU instructions
#include <arm_math.h>	 // Include CMSIS header
#include "MPU9250_t.h"




/********************************* Functions *********************************/
void initAllMPU9250Filters(void);
void filterAccelMPU9250(MPU9250_t* myMPU9250);

/**
 * @}
 */
/* end - MPU9250_FIR_Header */

#endif // MPUFILTER_H_
