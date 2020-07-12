#include <avr/io.h>
#include "binary_protocol.h"
#include "processor_data.h"
#include "flash.h"
#include "cpu.h"

/************************************************************************/
/* Extern functions and variables                                       */
/************************************************************************/
extern void set_boot_state_to_standby(void);
extern void set_boot_state_to_programming(void);

/************************************************************************/
/* parse()                                                              */
/************************************************************************/
protocol_t parse(protocol_t command)
{
	protocol_t reply = command;

	switch(command.op_code)
	{
		case OP_CODE_WRITE_PAGE:
			if ((command.address % SPM_PAGESIZE != 0) || (command.address > APP_SECTION_SIZE - SPM_PAGESIZE))
			{
				reply.error = ERROR_ON_ADDRESS;
				break;
			}
			if (command.data_length != SPM_PAGESIZE)
			{
				reply.error = ERROR_ON_DATA_LENGTH;
				break;
			}

			Flash_ProgramPage(command.address, command.data, 1);
			
			set_boot_state_to_programming();

			break;
		
		case OP_CODE_WRITE_EEPROM:
			if (command.address > EEPROM_SIZE)
			{
				reply.error = ERROR_ON_ADDRESS;
				break;
			}
			if ((command.address + command.data_length > EEPROM_SIZE) || (command.data_length > SPM_PAGESIZE))
			{
				reply.error = ERROR_ON_DATA_LENGTH;
				break;
			}

			for (uint32_t i = 0; i < command.data_length; i++)
				eeprom_wr_byte(command.address + i, command.data[i]);
				
			break;

		case OP_CODE_READ_EEPROM:
			if (command.address > EEPROM_SIZE)
			{
				reply.error = ERROR_ON_ADDRESS;
				break;
			}
			if ((command.address + command.data_length > EEPROM_SIZE) || (command.data_length > SPM_PAGESIZE))
			{
				reply.error = ERROR_ON_DATA_LENGTH;
				break;
			}

			for (uint32_t i = 0; i < command.data_length; i++)
				reply.data[i] = eeprom_rd_byte(command.address + i);
				
			break;
			
		case OP_CODE_GET_PAGE_SIZE:
            reply.data_length = (uint32_t)(SPM_PAGESIZE);
			break;
			
		case OP_CODE_LEAVE_BOOTLOADER:
			set_boot_state_to_standby();

			break;
	}
	
	return reply;
}