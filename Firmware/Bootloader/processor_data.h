#ifndef _PROCESSOR_DATA_H_
#define _PROCESSOR_DATA_H_

/************************************************************************/
/* Serial ports                                                         */
/************************************************************************/
#if defined(__AVR_ATxmega16A4U__)
	// -Wl,--section-start=.text=0x4000
	#define UART				USARTE0
	#define UART_PORT			PORTE
	#define UART_RX_pin		2
	#define UART_TX_pin		3

	#define LED_PORT			PORTD
	#define LED_PORT_pin		5
#endif

#if defined(__AVR_ATxmega32A4U__)
	// -Wl,--section-start=.text=0x8000
	#define UART				USARTE0
	#define UART_PORT			PORTE
	#define UART_RX_pin		2
	#define UART_TX_pin		3

	#define LED_PORT			PORTR
	#define LED_PORT_pin		0
#endif

#if defined(__AVR_ATxmega64A4U__)
	// -Wl,--section-start=.text=0x10000
	#define UART				USARTE0
	#define UART_PORT			PORTE
	#define UART_RX_pin		2
	#define UART_TX_pin		3

	#define LED_PORT			PORTD
	#define LED_PORT_pin		5
#endif

#if defined(__AVR_ATxmega128A4U__)
	// -Wl,--section-start=.text=0x20000
	#define UART				USARTE0
	#define UART_PORT			PORTE
	#define UART_RX_pin		2
	#define UART_TX_pin		3

	#define LED_PORT			PORTR
	#define LED_PORT_pin		0
#endif

#if defined(__AVR_ATxmega128A1U__)
	// -Wl,--section-start=.text=0x20000
	#define UART				USARTF0
	#define UART_PORT			PORTF
	#define UART_RX_pin		2
	#define UART_TX_pin		3

	#define LED_PORT			PORTA
	#define LED_PORT_pin		6
#endif

/************************************************************************/
/* Macros                                                               */
/************************************************************************/
#define set_LED   set_io(LED_PORT, LED_PORT_pin)
#define clr_LED   clear_io(LED_PORT, LED_PORT_pin)
#define tgl_LED   toggle_io(LED_PORT, LED_PORT_pin)

#endif /* _PROCESSOR_DATA_H_ */