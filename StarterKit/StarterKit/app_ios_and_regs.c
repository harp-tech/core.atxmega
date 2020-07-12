#include <avr/io.h>
#include "hwbp_core_types.h"
#include "app_ios_and_regs.h"

/************************************************************************/
/* Configure and initialize IOs                                         */
/************************************************************************/
void init_ios(void)
{	/* Configure input pins */
	io_pin2in(&PORTB, 1, PULL_IO_UP, SENSE_IO_EDGES_BOTH);               // INPUT

	/* Configure input interrupts */
	io_set_int(&PORTB, INT_LEVEL_LOW, 0, (1<<1), false);                 // INPUT

	/* Configure output pins */
	io_pin2out(&PORTB, 0, OUT_IO_DIGITAL, IN_EN_IO_EN);                  // OUTPUT

	/* Initialize output pins */
	clr_OUTPUT;
}

/************************************************************************/
/* Registers' stuff                                                     */
/************************************************************************/
AppRegs app_regs;

uint8_t app_regs_type[] = {
	TYPE_U8,
	TYPE_I16
};

uint16_t app_regs_n_elements[] = {
	1,
	16
};

uint8_t *app_regs_pointer[] = {
	(uint8_t*)(&app_regs.REG_DUMMY0),
	(uint8_t*)(app_regs.REG_DUMMY1)
};