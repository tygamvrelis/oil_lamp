/**
 * @file table.c
 * @author Tyler
 *
 * @ingroup Table
 * @{
 */




/********************************** Includes *********************************/
#include <App/table.h>
#include "cmsis_os.h"




/***************************** Extern declarations ***************************/
extern osSemaphoreId TableLockHandle;




/****************************** Private variables ****************************/
static float table[MAX_TABLE_IDX] = {0};




/***************************** Public Functions ******************************/
bool write_table(idx_t idx, float value)
{
	if (idx >= MAX_TABLE_IDX)
	{
		return false;
	}

	xSemaphoreTake(TableLockHandle, pdMS_TO_TICKS(1));
	table[idx] = value;
	xSemaphoreGive(TableLockHandle);

	return true;
}

bool read_table(idx_t idx, float* data)
{
	if (idx >= MAX_TABLE_IDX)
	{
		return false;
	}

	xSemaphoreTake(TableLockHandle, pdMS_TO_TICKS(1));
	*data = table[idx];
	xSemaphoreGive(TableLockHandle);

	return true;
}

/**
 * @}
 */
/* end - Table */
