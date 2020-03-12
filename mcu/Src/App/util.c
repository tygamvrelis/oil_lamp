/**
 * @file util.c
 * @author Tyler
 *
 * @ingroup Util
 * @{
 */




/********************************** Includes *********************************/
#include "App/util.h"




/***************************** Public Functions ******************************/
int8_t bound_int8_t(int8_t val, int8_t min_val, int8_t max_val)
{
    if (val < min_val)
    {
        return min_val;
    }
    else if (val > max_val)
    {
        return max_val;
    }
    else
    {
        return val;
    }
}

//-----------------------------------------------------------------------------

float bound_float(float val, float min_val, float max_val)
{
    if (val < min_val)
    {
        return min_val;
    }
    else if (val > max_val)
    {
        return max_val;
    }
    else
    {
        return val;
    }
}

//-----------------------------------------------------------------------------

uint8_t rs232_checksum(uint8_t* arr, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; ++i)
    {
        sum += arr[i];
    }
    return (~sum) & 0xFF;
}

/**
 * @}
 */
/* end - Util */
