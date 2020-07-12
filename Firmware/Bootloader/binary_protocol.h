#ifndef _BINARY_PROTOCOL_H_
#define _BINARY_PROTOCOL_H_
#include <avr/io.h>

/*
== Bootloader Protocol ==

Master: [01] [02] [03]   [Op Code] [Error]   [Address (4 Bytes)] [Data Length (4 Bytes)]   [[Data] (n)]   [Checksum (16 bits)]
Slave:  [01] [02] [03]   [Op Code] [Error]   [Address (4 Bytes)] [Data Length (4 Bytes)]   [[Data] (n)]   [Checksum (16 bits)]


Command
   [00] Write Page
   [01] Read Page (not implemented on firmware)
   [02] Write EEPROM
   [03] Read EEPROM
   [66] Get Page Size of the device (the field Data is not used)
   [77] Leave Bootloader and go to Application Code

Error
   [00] No error
   [01] Generic error
   [02] Address not correct
   [03] Data Length not correct

Data
   The length of Data is fixed and depends on the microcontroller family.
      -> ATXMega : 512 bytes

   If the Data bytes are not all used, the used bytes should appear in the lowest index positions of the Data vector.

*/

/************************************************************************/
/* Protocol                                                             */
/************************************************************************/
#define OP_CODE_WRITE_PAGE				0x00
#define OP_CODE_WRITE_EEPROM			0x02
#define OP_CODE_READ_EEPROM			0x03
#define OP_CODE_GET_PAGE_SIZE			0x66
#define OP_CODE_LEAVE_BOOTLOADER		0x77

#define NO_ERROR					0
#define GENERIC_ERROR			1
#define ERROR_ON_ADDRESS		2
#define ERROR_ON_DATA_LENGTH	3

typedef struct
{
	uint8_t header[3];
	uint8_t op_code;
	uint8_t error;
	uint32_t address;
	uint32_t data_length;
	uint8_t data[SPM_PAGESIZE];
	uint16_t checksum;
} protocol_t;

#endif /* _BINARY_PROTOCOL_H_ */