#include "hwbp_core_com.h"


/************************************************************************/
/* Definition of external variables                                     */
/************************************************************************/
uint8_t com_mode = COM_MODE_UART;	// Reflects the actual mode (UART or USB)
uint8_t rx_timeout = 0;					// Countdown for timeout
uint8_t rx_cmd_ready = 0;				// Contains the buffer index of the available command on rxbuff_hwbp_uart_buffX or rxbuff_hwbp_usbX
uint8_t cmd_len_buff1;					// Contains the "len" of the command available on buffer 1
uint8_t cmd_len_buff2;					// Contains the "len" of the command available on buffer 2


/************************************************************************/
/* Buffers and pointers                                                 */
/************************************************************************/
uint8_t txbuff_hwbp_uart[HWBP_UART_TXBUFSIZ];
uint8_t rxbuff_hwbp_uart_buff1[HWBP_UART_RXBUFSIZ];
uint8_t rxbuff_hwbp_uart_buff2[HWBP_UART_RXBUFSIZ];

#if HWBP_UART_TXBUFSIZ >= 256
	uint16_t hwbp_uart_tail = 0;
	uint16_t hwbp_uart_head = 0;
#else
	uint8_t hwbp_uart_tail = 0;
	uint8_t hwbp_uart_head = 0;
#endif

#if HWBP_UART_RXBUFSIZ >= 256
	uint16_t hwbp_uart_rx_pointer_buff1 = 0;
	uint16_t hwbp_uart_rx_pointer_buff2 = 0;
#else
	uint8_t hwbp_uart_rx_pointer_buff1 = 0;
	uint8_t hwbp_uart_rx_pointer_buff2 = 0;
#endif

bool receiving_on_buff1 = true;

	
/************************************************************************/
/* Initialization and ON/OFF                                            */
/************************************************************************/
void hwbp_com_uart_init(uint16_t BSEL, int8_t BSCALE, bool use_clk2x)
{
	#ifdef HWBP_UART_USE_FLOW_CONTROL
		io_pin2out(&HWBP_UART_RTS_PORT, HWBP_UART_RTS_pin, OUT_IO_DIGITAL, IN_EN_IO_DIS);
		io_pin2in(&HWBP_UART_CTS_PORT, HWBP_UART_CTS_pin, PULL_IO_TRISTATE, SENSE_IO_EDGES_BOTH);
		io_set_int(&HWBP_UART_CTS_PORT, HWBP_UART_CTS_INT_LEVEL, HWBP_UART_CTS_INT_N, (1<<HWBP_UART_CTS_pin), true);
		enable_hwbp_uart_rx;
	#endif
	
	HWBP_UART_UART.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
	HWBP_UART_UART.BAUDCTRLA = *((uint8_t*)&BSEL);
	HWBP_UART_UART.BAUDCTRLB = (*(1+(uint8_t*)&BSEL) & 0x0F) | ((BSCALE<<4) & 0xF0);

	if (use_clk2x)
		HWBP_UART_UART.CTRLB |= USART_CLK2X_bm;
	
	set_io(HWBP_UART_PORT, HWBP_UART_TX_pin);
	io_pin2out(&HWBP_UART_PORT, HWBP_UART_TX_pin, OUT_IO_DIGITAL, IN_EN_IO_DIS);
	io_pin2in(&HWBP_UART_PORT, HWBP_UART_RX_pin, PULL_IO_TRISTATE, SENSE_IO_NO_INT_USED);
	
	#ifdef HWBP_UART_USE_FLOW_CONTROL
		if (HWBP_UART_CTS_PORT.IN & (1 << HWBP_UART_CTS_pin) )
			/* Disable uart interrupt until RTS is logic low */
			HWBP_UART_UART.CTRLA &= ~(USART_DREINTLVL_OFF_gc | USART_DREINTLVL_gm);
	#endif
}

void hwbp_com_uart_enable(void)
{
	HWBP_UART_UART.CTRLB |= (USART_RXEN_bm | USART_TXEN_bm);
	HWBP_UART_UART.STATUS = USART_RXCIF_bm | USART_TXCIF_bm | USART_DREIF_bm;
	HWBP_UART_UART.CTRLA |= (HWBP_UART_RX_INT_LEVEL<< 4);
}

void hwbp_com_uart_disable(void)
{
	HWBP_UART_UART.CTRLB &= (USART_RXEN_bm | USART_TXEN_bm);
}

/************************************************************************/
/* Interrupt TX                                                         */
/************************************************************************/
HWBP_UART_TX_ROUTINE_
{
	//core_callback_uart_tx_before_exec();

	HWBP_UART_UART.DATA = txbuff_hwbp_uart[hwbp_uart_tail++];
	if (hwbp_uart_tail == HWBP_UART_TXBUFSIZ)
		hwbp_uart_tail = 0;
	
	/* disable this interrupt until new data arrive to buffer */
	if (hwbp_uart_head == hwbp_uart_tail)
		HWBP_UART_UART.CTRLA &= ~(USART_DREINTLVL_OFF_gc | USART_DREINTLVL_gm);
	
	//core_callback_uart_tx_after_exec();
	hwbp_uart_leave_interrupt;
}

/************************************************************************/
/* Interrupt CTS                                                        */
/************************************************************************/
HWBP_UART_CTS_ROUTINE_
{	
	//core_callback_uart_cts_before_exec();

	if (HWBP_UART_CTS_PORT.IN & (1 << HWBP_UART_CTS_pin) )
		/* Disable uart interrupt until RTS is logic low */
		HWBP_UART_UART.CTRLA &= ~(USART_DREINTLVL_OFF_gc | USART_DREINTLVL_gm);
	else
		if (hwbp_uart_tail != hwbp_uart_head)
			/* If the buffer is not empty, enable Tx interrupt */
			HWBP_UART_UART.CTRLA |= HWBP_UART_TX_INT_LEVEL;

	//core_callback_uart_cts_after_exec();
	hwbp_uart_leave_interrupt;
}

/************************************************************************/
/* Send data                                                            */
/************************************************************************/
void hwbp_uart_xmit_now(const uint8_t *dataIn0, uint8_t siz)
{
	for (uint8_t i = 0; i < siz; i++) {
		loop_until_bit_is_set(HWBP_UART_UART.STATUS, USART_DREIF_bp);
		HWBP_UART_UART.DATA = dataIn0[i];
	}
}

void hwbp_uart_xmit_now_byte(const uint8_t byte)
{
	loop_until_bit_is_set(HWBP_UART_UART.STATUS, USART_DREIF_bp);
	HWBP_UART_UART.DATA = byte;
}

void hwbp_uart_xmit(const uint8_t *dataIn0, uint8_t siz)
{
	#ifdef HWBP_UART_USE_FLOW_CONTROL
		if (!(HWBP_UART_CTS_PORT.IN & (1 << HWBP_UART_CTS_pin)))
	#endif
			HWBP_UART_UART.CTRLA |= HWBP_UART_TX_INT_LEVEL;	// Re-enable TX interrupt
	
	
	uint16_t space = HWBP_UART_TXBUFSIZ - hwbp_uart_head;
	if (space >= siz)
	{
		memcpy(txbuff_hwbp_uart+hwbp_uart_head, dataIn0, siz);

		/*
		uint8_t *pi; 
		uint8_t *po; 
		uint8_t n; 

		pi=txbuff_hwbp_uart+hwbp_uart_head; 
		po=dataIn0;
		n=siz;
		while(n--){
			*pi++ = *po++;
		}
		*/
		
		bool tail_ahead = (hwbp_uart_tail > hwbp_uart_head);
		hwbp_uart_head += siz;
		if (hwbp_uart_head == HWBP_UART_TXBUFSIZ)
		{
			hwbp_uart_head = 0;
			if (hwbp_uart_tail == 0)    hwbp_uart_tail = 1;			// lose oldest byte in buffer
		}
		else if (tail_ahead && hwbp_uart_tail <= hwbp_uart_head)	// if buffer overflow
		{
			hwbp_uart_tail = hwbp_uart_head+1;								// lose oldest bytes in buffer
			if (hwbp_uart_tail == HWBP_UART_TXBUFSIZ)    hwbp_uart_tail = 0;
			
		}
	}
	else
	{
		memcpy(txbuff_hwbp_uart+hwbp_uart_head, dataIn0, space);
		siz -= space;
		memcpy(txbuff_hwbp_uart, dataIn0+space, siz);
		bool tail_ahead = (hwbp_uart_tail > hwbp_uart_head);
		hwbp_uart_head = siz;
		if (tail_ahead || hwbp_uart_tail <= hwbp_uart_head)			// if buffer overflow
		{
			hwbp_uart_tail = hwbp_uart_head+1;								// lose oldest bytes in buffer
			if (hwbp_uart_tail == HWBP_UART_TXBUFSIZ)    hwbp_uart_tail = 0;
		}
	}
}

/************************************************************************/
/* Receive data                                                         */
/************************************************************************/
bool hwbp_uart_rcv_now(uint8_t * byte)
{
	if (HWBP_UART_UART.STATUS & USART_RXCIF_bm)
	{
		*byte = HWBP_UART_UART.DATA;
		return true;
	}
	
	return false;
}

extern void core_func_catastrophic_error_detected(void);

HWBP_UART_RX_ROUTINE_
{
	uint8_t chr;
	//core_callback_uart_rx_before_exec();
	chr = HWBP_UART_UART.DATA;
	
	if (receiving_on_buff1)
	{
		rxbuff_hwbp_uart_buff1[hwbp_uart_rx_pointer_buff1++] = chr;

		if (hwbp_uart_rx_pointer_buff1 == 1)
		{
			if (rxbuff_hwbp_uart_buff1[0] != 0x02 && rxbuff_hwbp_uart_buff1[0] != 0x01)
				hwbp_uart_rx_pointer_buff1 = 0;
			else
				rx_timeout = RX_TIMEOUT_MS;
		}

		else if (hwbp_uart_rx_pointer_buff1 == 4)
		{
			if (rxbuff_hwbp_uart_buff1[1] != 255)
			{
				if(rxbuff_hwbp_uart_buff1[1] + 2 < HWBP_UART_RXBUFSIZ)
					cmd_len_buff1 = rxbuff_hwbp_uart_buff1[1] + 2;
				else
					hwbp_uart_rx_pointer_buff1 = 0;
			}
			else
			{
				if (*((uint16_t*)(rxbuff_hwbp_uart_buff1+1)) + 4 < HWBP_UART_RXBUFSIZ)
					cmd_len_buff1 = *((uint16_t*)(rxbuff_hwbp_uart_buff1+2)) + 4;
				else
					hwbp_uart_rx_pointer_buff1 = 0;
			}
		}

		else if (hwbp_uart_rx_pointer_buff1 == cmd_len_buff1)
		{
			/* Make sure that UART or USB interrupts will not
			update rx buffers until command is processed */
			if (com_mode == COM_MODE_UART)
			{
				/* Disable rx external hardware */
				disable_hwbp_uart_rx;

	 			/* Disable high level interrupts */
	 			PMIC_CTRL = PMIC_RREN_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;
	 			
	 			/* Disable USART RX */
	 			HWBP_UART_UART.CTRLA &= ~(USART_RXCINTLVL_gm);
	 
	 			/* Re-enable high level interrupts */
	 			PMIC_CTRL = PMIC_RREN_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
			}

			/* Remove "header" and "len" from cmd_len */
			cmd_len_buff1 -= (rxbuff_hwbp_uart_buff1[1] != 255) ? 2 : 4;
		
			hwbp_uart_rx_pointer_buff1 = 0;

			rx_cmd_ready = 1;	// Ready on buffer1
			rx_timeout = 0;
			receiving_on_buff1 = false;
		}
	}
	
	else
	
	{
		rxbuff_hwbp_uart_buff2[hwbp_uart_rx_pointer_buff2++] = chr;

		if (hwbp_uart_rx_pointer_buff2 == 1)
		{
			if (rxbuff_hwbp_uart_buff2[0] != 0x02 && rxbuff_hwbp_uart_buff2[0] != 0x01)
				hwbp_uart_rx_pointer_buff2 = 0;
			else
				rx_timeout = RX_TIMEOUT_MS;
		}
			
		else if (hwbp_uart_rx_pointer_buff2 == 4)
		{
			if (rxbuff_hwbp_uart_buff2[1] != 255)
			{
				if(rxbuff_hwbp_uart_buff2[1] + 2 < HWBP_UART_RXBUFSIZ)
					cmd_len_buff2 = rxbuff_hwbp_uart_buff2[1] + 2;
				else
					hwbp_uart_rx_pointer_buff2 = 0;
			}
			else
			{
				if (*((uint16_t*)(rxbuff_hwbp_uart_buff2+1)) + 4 < HWBP_UART_RXBUFSIZ)
					cmd_len_buff2 = *((uint16_t*)(rxbuff_hwbp_uart_buff2+2)) + 4;
				else
					hwbp_uart_rx_pointer_buff2 = 0;
			}
		}

		else if (hwbp_uart_rx_pointer_buff2 == cmd_len_buff2)
		{
			/* Make sure that UART or USB interrupts will not
			update rx buffers until command is processed */
			if (com_mode == COM_MODE_UART)
			{
				/* Disable rx external hardware */
				disable_hwbp_uart_rx;

	 			/* Disable high level interrupts */
	 			PMIC_CTRL = PMIC_RREN_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;
	 			
	 			/* Disable USART RX */
	 			HWBP_UART_UART.CTRLA &= ~(USART_RXCINTLVL_gm);
	 
	 			/* Re-enable high level interrupts */
	 			PMIC_CTRL = PMIC_RREN_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
			}

			/* Remove "header" and "len" from cmd_len */
			cmd_len_buff2 -= (rxbuff_hwbp_uart_buff2[1] != 255) ? 2 : 4;
		
			hwbp_uart_rx_pointer_buff2 = 0;

			rx_cmd_ready = 2;	// Ready on buffer2
			rx_timeout = 0;
			receiving_on_buff1 = true;
		}
	}
	
	//core_callback_uart_rx_after_exec();
	hwbp_uart_leave_interrupt;
}