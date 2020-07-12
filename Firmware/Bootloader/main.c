#include "cpu.h"
#include "binary_protocol.h"
#include "processor_data.h"
#include "parse.h"
#include "flash.h"
#define F_CPU 32000000UL
#include <util/delay.h>

/************************************************************************/
/* Receiver state definitions                                           */
/************************************************************************/
#define RX_STATE_BYTE0				0
#define RX_STATE_BYTE1				1
#define RX_STATE_BYTE2				2
#define RX_STATE_COMMAND			3
#define RX_STATE_ERROR				4
#define RX_STATE_DATA_AND_CHECK	5

/************************************************************************/
/* Bootloader state control                                             */
/************************************************************************/
#define BOOT_STATE_EEPROM_ADDRESS (EEPROM_SIZE - 1)
#define BOOT_STATE_STANDBY        0xFF
#define BOOT_STATE_PROGRAMMING    0x11

uint8_t boot_state;

void set_boot_state_to_standby(void)
{
	if (boot_state != BOOT_STATE_STANDBY)
	{
		eeprom_wr_byte(BOOT_STATE_EEPROM_ADDRESS, BOOT_STATE_STANDBY);
	}
    boot_state = BOOT_STATE_STANDBY;
}

void set_boot_state_to_programming(void)
{
	if (boot_state != BOOT_STATE_PROGRAMMING)
	{
		eeprom_wr_byte(BOOT_STATE_EEPROM_ADDRESS, BOOT_STATE_PROGRAMMING);
	}
    boot_state = BOOT_STATE_PROGRAMMING;
}

void load_boot_state(void)
{
	boot_state = eeprom_rd_byte(BOOT_STATE_EEPROM_ADDRESS);
}

/************************************************************************/
/* main()                                                               */
/************************************************************************/
int main(void)
{
	/* Variables */
	protocol_t command_received;	
	protocol_t reply;
	uint32_t time_counter = 0;

	/* Initialize command_received's header */	
	command_received.header[0] = 1;
	command_received.header[1] = 2;
	command_received.header[2] = 3;
	
	/* UART with 1 Mbit/s */
	uint16_t BSEL = 1;
	int8_t BSCALE = 0;
	bool use_clk2x = false;

	/* Shutdown watchdog */
	wdt_disable();
	
	/* Start external clock */
	cpu_config_clock(F_CPU, true, true);

	/* Load boot current state */
	load_boot_state();

	/* Configure LED pin */
	io_pin2out(&LED_PORT, LED_PORT_pin, OUT_IO_DIGITAL, IN_EN_IO_DIS);
	set_LED;

	/* Configure UART pins */
	set_io(UART_PORT, UART_TX_pin);
	io_pin2out(&UART_PORT, UART_TX_pin, OUT_IO_DIGITAL, IN_EN_IO_DIS);
	io_pin2in(&UART_PORT, UART_RX_pin, PULL_IO_TRISTATE, SENSE_IO_NO_INT_USED);
	
	/* Configure UART peripheral */
	UART.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
	UART.BAUDCTRLA = *((uint8_t*)&BSEL);
	UART.BAUDCTRLB = (*(1+(uint8_t*)&BSEL) & 0x0F) | ((BSCALE<<4) & 0xF0);
	if (use_clk2x)
		UART.CTRLB |= USART_CLK2X_bm;
	UART.CTRLB |= (USART_RXEN_bm | USART_TXEN_bm);
	
	/* Receive bytes and detect the protocol */
	uint8_t byte;
	uint8_t rx_state = RX_STATE_BYTE0;
	uint16_t auxiliar_index;
	uint16_t checksum;

	/* Definitions used for time counter */
	#define _3_SECS    880000     // ~3 seconds
	#define _100_MSECS _3_SECS/30
	
	/*
	_delay_ms(500);
	set_LED;
	_delay_ms(500);
	tgl_LED;
	_delay_ms(500);
	set_LED;
	_delay_ms(500);
	tgl_LED;
    */
	

	/* Main loop */
	while(1)
	{
		/* Search for when to set the LED */
		if ((time_counter == _100_MSECS*0)  || \
		    (time_counter == _100_MSECS*2)  || \
		    (time_counter == _100_MSECS*10) || \
		    (time_counter == _100_MSECS*12) || \
		    (time_counter == _100_MSECS*20) || \
		    (time_counter == _100_MSECS*22))
		{
			set_LED;

			/* Rx timeout equal to 1 second */
			if (time_counter == _100_MSECS*10)
			{
				rx_state = RX_STATE_BYTE0;
			}
		}
		
		/* Search for when to clear the LED */
		if ((time_counter == _100_MSECS*(0+1))  || \
		    (time_counter == _100_MSECS*(2+1))  || \
		    (time_counter == _100_MSECS*(10+1)) || \
		    (time_counter == _100_MSECS*(12+1)) || \
		    (time_counter == _100_MSECS*(20+1)) || \
		    (time_counter == _100_MSECS*(22+1)))
		{
			clr_LED;
		}

		/* Check the 3 seconds bootloader timeout */
		if (++time_counter == _3_SECS)
		{
			if (boot_state == BOOT_STATE_STANDBY)
			{
				asm volatile ("jmp 0");
			}
			else
			{
				time_counter = 0;
			}
		}

		/* Check for a new byte from the serial */
		if (UART.STATUS & USART_RXCIF_bm)
		{
			byte = UART.DATA;
			time_counter = 0;

			switch (rx_state)
			{
				case RX_STATE_BYTE0:
					if (byte == 1)
						rx_state = RX_STATE_BYTE1;
					else
						rx_state = RX_STATE_BYTE0;
					break;

				case RX_STATE_BYTE1:
					if (byte == 2)
						rx_state = RX_STATE_BYTE2;
					else
						rx_state = RX_STATE_BYTE0;
					break;
				
				case RX_STATE_BYTE2:
					if (byte == 3)
						rx_state = RX_STATE_COMMAND;
					else
						rx_state = RX_STATE_BYTE0;
					break;
				
				case RX_STATE_COMMAND:
					command_received.op_code = byte;
					rx_state = RX_STATE_ERROR;
					break;
				
				case RX_STATE_ERROR:
					command_received.error = byte;
					rx_state = RX_STATE_DATA_AND_CHECK;
                    
                    if (command_received.op_code == OP_CODE_GET_PAGE_SIZE)
                    {					
					    auxiliar_index = 0;                        
                        while (auxiliar_index != 4 + 4) 
					    {
						    if (UART.STATUS & USART_RXCIF_bm)
						    {
							    *(((uint8_t*)(&command_received.address)) + auxiliar_index++) = UART.DATA;
						    }
					    }
                        
                        auxiliar_index = 0;
                        while (auxiliar_index != 2) 
					    {
						    if (UART.STATUS & USART_RXCIF_bm)
						    {
							    *(((uint8_t*)(&command_received.checksum)) + auxiliar_index++) = UART.DATA;
						    }
					    }
                        
                        checksum = 0;
                        for (uint8_t i = 0; i < 3 + 1 + 1 + 4 + 4; i++)
                        {
                            checksum += *(((uint8_t*)(&command_received.header)) + i);
                        }
                        
                        if ((checksum == command_received.checksum) && (command_received.error == NO_ERROR))
                        {
                            reply = parse(command_received);
                            
                            reply.checksum = 0;
                            for (uint8_t i = 0; i < 3 + 1 + 1 + 4 + 4; i++)
                            {
                                loop_until_bit_is_set(UART.STATUS, USART_DREIF_bp);
                                UART.DATA = *(((uint8_t*)(&reply.header)) + i);

                                reply.checksum += *(((uint8_t*)(&reply.header)) + i);
                            }
                            
                            loop_until_bit_is_set(UART.STATUS, USART_DREIF_bp);
                            UART.DATA = *(((uint8_t*)(&reply.checksum)) + 0);
                            //UART.DATA = 1;

                            loop_until_bit_is_set(UART.STATUS, USART_DREIF_bp);
                            UART.DATA = *(((uint8_t*)(&reply.checksum)) + 1);
                            //UART.DATA = 2;
                        }
                    
                        rx_state = RX_STATE_BYTE0;
                    }
                    
					break;

				case RX_STATE_DATA_AND_CHECK:					
					auxiliar_index = 0;
					
					tgl_LED;
					
					*(((uint8_t*)(&command_received.address)) + auxiliar_index++) = byte;

					while (auxiliar_index != 4 + 4 + APP_SECTION_PAGE_SIZE + 2) 
					{
						if (UART.STATUS & USART_RXCIF_bm)
						{
							*(((uint8_t*)(&command_received.address)) + auxiliar_index++) = UART.DATA;
						}
					}

					checksum = 0;
					for (uint16_t i = 0; i < 3 + 1 + 1 + 4 + 4 + APP_SECTION_PAGE_SIZE ; i++)
					{
						checksum += *(((uint8_t*)(&command_received.header)) + i);
					}

					if (checksum == command_received.checksum)
					{
						if (command_received.error == NO_ERROR)
                        {
                            reply = parse(command_received);
						
						    reply.checksum = 0;
						    for (uint16_t i = 0; i < 3 + 1 + 1 + 4 + 4 + APP_SECTION_PAGE_SIZE ; i++)
						    {							
							    loop_until_bit_is_set(UART.STATUS, USART_DREIF_bp);
							    UART.DATA = *(((uint8_t*)(&reply.header)) + i);

							    reply.checksum += *(((uint8_t*)(&reply.header)) + i);
						    }

						    loop_until_bit_is_set(UART.STATUS, USART_DREIF_bp);
						    UART.DATA = *(((uint8_t*)(&reply.checksum)) + 0);

						    loop_until_bit_is_set(UART.STATUS, USART_DREIF_bp);
						    UART.DATA = *(((uint8_t*)(&reply.checksum)) + 1);

						    if ((reply.op_code == OP_CODE_LEAVE_BOOTLOADER) && (reply.error == NO_ERROR))
						    {
							    loop_until_bit_is_set(UART.STATUS, USART_DREIF_bp);
							    asm volatile ("jmp 0");
						    }
                        }                            
					}

					rx_state = RX_STATE_BYTE0;
					break;

			}
		}
	}
	

}