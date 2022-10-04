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
 * @return the checksum of the given byte array
 * @param arr the array of bytes to compute the checksum over
 * @param len the length of the array
 */
uint8_t rs232_checksum(uint8_t* arr, uint8_t len);

/**
 * @}
 */
/* end - Util */

#endif /* UTIL_H_ */
