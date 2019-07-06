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

/**
 * @}
 */
/* end - Util */
