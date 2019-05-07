/**
 * @file MPUFilter.c
 * @author Tyler
 * @brief API for filtering accelerometer data. Filter coefficients &
 *        preliminary C code was generated with MicroModeler DSP
 *
 * @defgroup MPU9250_FIR FIR Filter
 * @brief Digitally filters accelerometer data
 * @ingroup MPU9250_Driver
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




/********************************** Includes *********************************/
#include "MPUFilter.h"
#include <string.h> // For memset




/********************************* Constants *********************************/
/** @brief Number of taps (coefficients) in the filter */
static const int MPUFilter_numTaps = 21;

/**
 * @brief Number of samples to collect before updating filter output. Not
 *        used in our application
 */
static const int MPUFilter_blockSize = 16;

/** @brief Filter coefficients */
static const float32_t MPUFilter_coefficients[21] = {-0.018872079, -0.0011102221,
        0.0030367336, 0.014906744, 0.030477359, 0.049086205, 0.070363952,
        0.089952103, 0.10482875, 0.11485946, 0.11869398, 0.11485946, 0.10482875,
        0.089952103, 0.070363952, 0.049086205, 0.030477359, 0.014906744,
        0.0030367336, -0.0011102221, -0.018872079};




/*********************************** Types ***********************************/
/** @brief Container for filter information */
typedef struct{
    arm_fir_instance_f32 instance; /**< Filter instance            */
    float32_t state[37];           /**< Current & buffered samples */
    float32_t output;              /**< Output of filter           */
} MPUFilterType;




/***************************** Private Variables *****************************/
/** @brief Filter data structures for x-, y-, and z-axis acceleration */
static MPUFilterType azFilter, ayFilter, axFilter;




/***************************** Private Functions *****************************/
/**
 * @defgroup MPU9250_FIR_Private_Functions Private functions
 * @brief Functions used interally
 * @ingroup MPU9250_FIR
 * @{
 */

/**
 * @brief Resets filter state
 * @param pThis Pointer to filter data structure
 */
static void MPUFilter_reset(MPUFilterType * pThis){
    memset(&pThis->state, 0, sizeof(pThis->state)); // Reset state to 0
    pThis->output = 0; // Reset output
}

/**
 * @brief Write a sample to the filter and update its output
 * @param pThis Pointer to filter data structure
 * @param input The new sample
 */
static inline void MPUFilter_writeInput(MPUFilterType * pThis, float input){
    arm_fir_f32(&pThis->instance, &input, &pThis->output, 1);
}

/**
 * @brief  Read the filter output
 * @param  pThis Pointer to filter data structure
 * @return The filter output
 */
static inline float MPUFilter_readOutput(MPUFilterType * pThis){
    return pThis->output;
}

/**
 * @brief Initializes a filter data structure
 * @param pThis Pointer to filter data structure
 */
static void MPUFilter_init(MPUFilterType * pThis){
    arm_fir_init_f32(
            &pThis->instance,
            MPUFilter_numTaps,
            (float32_t*)MPUFilter_coefficients,
            pThis->state,
            MPUFilter_blockSize
    );

    MPUFilter_reset(pThis);
}

/**
 * @brief Fills the filter with dummy inputs so that when the system starts
 *        we will not accidentally start the experiment while the filter is
 *        "warming up" with samples
 * @param pThis Pointer to filter data structure
 * @param dummyInput The value to fill the filter buffer with
 */
static void MPUFilter_writeDummyData(MPUFilterType* pThis, float dummyInput){
    for(uint8_t i = 0; i < MPUFilter_numTaps; ++i){
        MPUFilter_writeInput(pThis, dummyInput);
    }
}

/**
 * @}
 */
/* end - MPU9250_FIR_Private_Functions */




/***************************** Public Functions ******************************/
/**
 * @defgroup MPU9250_FIR_Public_Functions Public functions
 * @brief Functions used externally
 * @ingroup MPU9250_FIR
 * @{
 */

/**
 * @brief  Initializes all filters for the accelerometer data
 * @return None
 */
void initAllMPU9250Filters(void){
    MPUFilter_init(&axFilter);
    MPUFilter_writeDummyData(&axFilter, 1.0);
    MPUFilter_init(&ayFilter);
    MPUFilter_writeDummyData(&ayFilter, 1.0);
    MPUFilter_init(&azFilter);
    MPUFilter_writeDummyData(&azFilter, 9.81);
}

/**
 * @brief Given fresh sensor data for acceleration in each axis, this function
 *        filters the data then writes in back into the data structure passed
 *        in
 * @param myMPU9250 Pointer to the data structure which stores the data read
 *        from the MPU9250 sensor
 */
void filterAccelMPU9250(MPU9250_t* myMPU9250){
    // Filter the signals along each axis
    MPUFilter_writeInput(&axFilter, myMPU9250->ax);
    myMPU9250->ax = MPUFilter_readOutput(&axFilter);

    MPUFilter_writeInput(&ayFilter, myMPU9250->ay);
    myMPU9250->ay = MPUFilter_readOutput(&ayFilter);

    MPUFilter_writeInput(&azFilter, myMPU9250->az);
    myMPU9250->az = MPUFilter_readOutput(&azFilter);
}

/**
 * @}
 */
/* end - MPU9250_FIR_Public_Functions */

/**
 * @}
 */
/* end - MPU9250_FIR */
