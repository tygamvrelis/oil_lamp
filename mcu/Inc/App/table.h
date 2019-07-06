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
#include "MPU6050/MPU6050_t.h"




/*********************************** Types ************************************/
typedef enum
{
	TABLE_IDX_BASE_DATA = 0,
	TABLE_IDX_LAMP_DATA = sizeof(imu_data_t),
	MAX_TABLE_IDX = 2 * sizeof(imu_data_t)
}idx_t;




/********************************* Functions *********************************/
/**
 * @brief Write num bytes from src into the table at the specified index
 */
bool write_table(idx_t idx, uint8_t* src, size_t num);

/**
 * @brief Read num bytes from the table at the specified index into dest
 */
bool read_table(idx_t idx, uint8_t* dest, size_t num);

/**
 * @}
 */
/* end - Table */

#endif /* TABLE_H_ */
