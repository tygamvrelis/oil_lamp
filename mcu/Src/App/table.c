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




/***************************** Extern declarations ***************************/
extern osSemaphoreId TableLockHandle;




/****************************** Private variables ****************************/
static uint8_t table[2 * sizeof(imu_data_t)] = {0};




/***************************** Public Functions ******************************/
bool write_table(idx_t idx, uint8_t* src, size_t num)
{
	if (num == 0 || src == NULL || (idx + num > sizeof(table)))
	{
		return false;
	}

	xSemaphoreTake(TableLockHandle, pdMS_TO_TICKS(1));
	memcpy(&table[idx], src, num);
	xSemaphoreGive(TableLockHandle);

	return true;
}

bool read_table(idx_t idx, uint8_t* dest, size_t num)
{
    if (num == 0 || dest == NULL || (idx + num > sizeof(table)))
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
