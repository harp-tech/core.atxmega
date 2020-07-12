#include "app_funcs.h"
#include "app_ios_and_regs.h"
#include "hwbp_core.h"


/************************************************************************/
/* Create pointers to functions                                         */
/************************************************************************/
extern AppRegs app_regs;

void (*app_func_rd_pointer[])(void) = {
	&app_read_REG_DUMMY0,
	&app_read_REG_DUMMY1
};

bool (*app_func_wr_pointer[])(void*) = {
	&app_write_REG_DUMMY0,
	&app_write_REG_DUMMY1
};


/************************************************************************/
/* REG_DUMMY0                                                           */
/************************************************************************/
void app_read_REG_DUMMY0(void)
{
	//app_regs.REG_DUMMY0 = 0;

}

bool app_write_REG_DUMMY0(void *a)
{
	uint8_t reg = *((uint8_t*)a);

	app_regs.REG_DUMMY0 = reg;
	return true;
}


/************************************************************************/
/* REG_DUMMY1                                                           */
/************************************************************************/
// This register is an array with 16 positions
void app_read_REG_DUMMY1(void)
{
	//app_regs.REG_DUMMY1[0] = 0;

}

bool app_write_REG_DUMMY1(void *a)
{
	int16_t *reg = ((int16_t*)a);

	app_regs.REG_DUMMY1[0] = reg[0];
	return true;
}