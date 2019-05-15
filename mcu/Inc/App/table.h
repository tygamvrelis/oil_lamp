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
#include <stddef.h>




/*********************************** Types ************************************/
typedef enum
{
	TABLE_IDX_BASE_DATA,
	TABLE_IDX_LAMP_DATA,
	MAX_TABLE_IDX
}idx_t;




/********************************* Functions *********************************/
bool write_table(idx_t idx, float* src, size_t num);
bool read_table(idx_t idx, float* dest, size_t num);

/**
 * @}
 */
/* end - Table */

#endif /* TABLE_H_ */
