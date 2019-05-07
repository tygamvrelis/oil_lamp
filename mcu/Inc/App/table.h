/**
 * @file table.h
 * @author Tyler
 *
 * @defgroup Table
 * @brief Global storage
 * @{
 */

#ifndef TABLE_H_
#define TABLE_H_




/********************************** Includes *********************************/
#include <stdbool.h>




/*********************************** Types ************************************/
typedef enum
{
	BASE_ANGLE_OUTER,
	BASE_ANGLE_INNER,
	LAMP_ANGLE_OUTER,
	LAMP_ANGLE_INNER,
	MAX_TABLE_IDX
}idx_t;




/********************************* Functions *********************************/
bool write_table(idx_t idx, float value);
bool read_table(idx_t idx, float* data);

/**
 * @}
 */
/* end - Table */

#endif /* TABLE_H_ */
