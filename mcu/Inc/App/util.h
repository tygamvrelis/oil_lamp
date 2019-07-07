/**
 * @file util.h
 * @author Tyler
 *
 * @defgroup Util
 * @brief Utilities
 * @{
 */

#ifndef UTIL_H_
#define UTIL_H_




/********************************** Includes *********************************/
#include <stdint.h>




/********************************* Functions *********************************/
/**
 * @return min_val if val < min_val, max_val if val > max_val, otherwise val
 */
int8_t bound_int8_t(int8_t val, int8_t min_val, int8_t max_val);

/**
 * @return min_val if val < min_val, max_val if val > max_val, otherwise val
 */
float bound_float(float val, float min_val, float max_val);

/**
 * @}
 */
/* end - Util */

#endif /* UTIL_H_ */
