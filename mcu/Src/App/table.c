/**
 * @file table.c
 * @author Tyler
 *
 * @ingroup Table
 * @{
 */




/********************************** Includes *********************************/
#include "App/table.h"
#include <string.h>
#include "cmsis_os.h"
#include "MPU6050/MPU6050_t.h"




/***************************** Extern declarations ***************************/
extern osSemaphoreId TableLockHandle;




/****************************** Private variables ****************************/
static imu_data_t table[MAX_TABLE_IDX] = {0};




/***************************** Public Functions ******************************/
bool write_table(idx_t idx, float* src, size_t num)
{
	if (num == 0 || src == NULL || idx * sizeof(imu_data_t) + num > sizeof(table))
	{
		return false;
	}

	xSemaphoreTake(TableLockHandle, pdMS_TO_TICKS(1));
	memcpy(&table[idx], src, num);
	xSemaphoreGive(TableLockHandle);

	return true;
}

bool read_table(idx_t idx, float* dest, size_t num)
{
	if (num == 0 || dest == NULL || idx * sizeof(imu_data_t) + num > sizeof(table))
	{
		return false;
	}

	xSemaphoreTake(TableLockHandle, pdMS_TO_TICKS(1));
	memcpy(dest, &table[idx], num);
	xSemaphoreGive(TableLockHandle);

	return true;
}

/**
 * @}
 */
/* end - Table */
