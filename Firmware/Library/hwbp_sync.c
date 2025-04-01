#include <avr/io.h>
#include "cpu.h"
#include "hwbp_sync.h"
#include "hwbp_core_regs.h"


/************************************************************************/
/* Global variables                                                     */
/************************************************************************/
#if defined(__AVR_ATxmega16A4U__)
    static uint16_t timestamp_tx_counter = 0;
    static uint8_t * timestamp_B0;
    static uint8_t * timestamp_B1;
    static uint8_t * timestamp_B2;
    static uint8_t * timestamp_B3;
#else
    static uint32_t sync_timestamp;
    static uint32_t* core_timestamp_pointer;
#endif

static uint32_t previous_timestamp = 0;

extern struct CommonBank commonbank;

bool clock_was_just_updated_externaly = false;


/*****************************************************************************/
/* Configure timestamp UART (100 Kbps), pins TX/RX and save SECOND pointers  */
/*****************************************************************************/
void initialize_timestamp_uart (uint32_t * timestamp_pointer)
{
    uint16_t BSEL = 19;
    int8_t BSCALE = 0;
    
    #if defined(__AVR_ATxmega16A4U__)    
        timestamp_B0 = (uint8_t*)(timestamp_pointer) + 0;
        timestamp_B1 = (uint8_t*)(timestamp_pointer) + 1;
        timestamp_B2 = (uint8_t*)(timestamp_pointer) + 2;
        timestamp_B3 = (uint8_t*)(timestamp_pointer) + 3;
    
        USARTD1_CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
        USARTD1_BAUDCTRLA = *((uint8_t*)&BSEL);
        USARTD1_BAUDCTRLB = (*(1+(uint8_t*)&BSEL) & 0x0F) | ((BSCALE<<4) & 0xF0);    
    
        io_pin2out(&PORTD, 7, OUT_IO_DIGITAL, IN_EN_IO_DIS);
        USARTD1_CTRLB = USART_TXEN_bm;          // Enable TX        
    #else    
        core_timestamp_pointer = timestamp_pointer;        
    #endif
    
    #if defined(__AVR_ATxmega32A4U__)
    
        USARTC0_CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
        USARTC0_BAUDCTRLA = *((uint8_t*)&BSEL);
        USARTC0_BAUDCTRLB = (*(1+(uint8_t*)&BSEL) & 0x0F) | ((BSCALE<<4) & 0xF0);
        
        io_pin2in(&PORTC, 2, PULL_IO_UP, SENSE_IO_EDGES_BOTH);
        USARTC0_CTRLB = USART_RXEN_bm;          // Enable RX
        USARTC0_CTRLA |= (INT_LEVEL_HIGH<< 4);  // High level interrupts for RX
        
    #endif
        
    
    #if defined(__AVR_ATxmega64A4U__)

        USARTD1_CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
        USARTD1_BAUDCTRLA = *((uint8_t*)&BSEL);
        USARTD1_BAUDCTRLB = (*(1+(uint8_t*)&BSEL) & 0x0F) | ((BSCALE<<4) & 0xF0);
         
        io_pin2in(&PORTD, 6, PULL_IO_UP, SENSE_IO_EDGES_BOTH);
        USARTD1_CTRLB = USART_RXEN_bm;          // Enable RX
        USARTD1_CTRLA |= (INT_LEVEL_HIGH<< 4);  // High level interrupts for RX
     
    #endif
    
    #if defined(__AVR_ATxmega128A1U__)
    
        USARTC1_CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
        USARTC1_BAUDCTRLA = *((uint8_t*)&BSEL);
        USARTC1_BAUDCTRLB = (*(1+(uint8_t*)&BSEL) & 0x0F) | ((BSCALE<<4) & 0xF0);    
    
        io_pin2in(&PORTC, 6, PULL_IO_UP, SENSE_IO_EDGES_BOTH);
        USARTC1_CTRLB = USART_RXEN_bm;          // Enable RX
        USARTC1_CTRLA |= (INT_LEVEL_HIGH<< 4);  // High level interrupts for RX
    
    #endif
    
    #if defined(__AVR_ATxmega128A4U__)
    
        USARTC0_CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
        USARTC0_BAUDCTRLA = *((uint8_t*)&BSEL);
        USARTC0_BAUDCTRLB = (*(1+(uint8_t*)&BSEL) & 0x0F) | ((BSCALE<<4) & 0xF0);
    
        io_pin2in(&PORTC, 2, PULL_IO_UP, SENSE_IO_EDGES_BOTH);
        USARTC0_CTRLB = USART_RXEN_bm;          // Enable RX
        USARTC0_CTRLA |= (INT_LEVEL_HIGH<< 4);  // High level interrupts for RX
    
    #endif    
}

/************************************************************************/
/* MASTER: Reset sync counter an trigger sync timer                     */
/************************************************************************/
void reset_sync_counter (void)
{
    #if defined(__AVR_ATxmega16A4U__)
    
        timestamp_tx_counter = 0;
    
    #endif
}

void trigger_sync_timer (void)
{
    #if defined(__AVR_ATxmega16A4U__)
    
        timestamp_tx_counter++;

        switch (timestamp_tx_counter)
        {
            case 1:
            case 2:
            case 4:
            case 6:
            case 7:
            case 1998:
            timer_type0_enable(&TCD0, TIMER_PRESCALER_DIV256, 44, INT_LEVEL_LOW);   /* 352 us = 512E-6 - 2 * (8/1 Kbps) */
        }
    
    #endif
}

/************************************************************************/
/* MASTER: xmit timestamp                                               */
/************************************************************************/
#if defined(__AVR_ATxmega16A4U__)
    ISR(TCD0_OVF_vect, ISR_NAKED)
    {   
        switch (timestamp_tx_counter)
        {
            case 1:
                USARTD1_DATA = 0xAA;
                break;
            case 2:
                USARTD1_DATA = 0xAF;
                break;
            case 4:
                USARTD1_DATA = *timestamp_B0;
                break;
            case 6:
                USARTD1_DATA = *timestamp_B1;
                break;
            case 7:
                USARTD1_DATA = *timestamp_B2;
                break;
            case 1998:
                USARTD1_DATA = *timestamp_B3;
                break;
        }
    
        timer_type0_stop(&TCD0);
    
        reti();
    }
#endif


/************************************************************************/
/* SLAVE: rcv timestamp                                                 */
/************************************************************************/
#if defined(__AVR_ATxmega16A4U__)
#else
    static uint8_t state = 0;
    static uint8_t converging;
    uint8_t device_lost_sync_counter = 254;	// Needs to be lower than 255
#endif

#if defined(__AVR_ATxmega32A4U__)
    #define USART_RX_BYTE USARTC0_DATA
    ISR(USARTC0_RXC_vect)
#endif

#if defined(__AVR_ATxmega64A4U__)
    #define USART_RX_BYTE USARTD1_DATA
    ISR(USARTD1_RXC_vect)
#endif

#if defined(__AVR_ATxmega128A1U__)
    #define USART_RX_BYTE USARTC1_DATA
    ISR(USARTC1_RXC_vect)
#endif

#if defined(__AVR_ATxmega128A4U__)
    #define USART_RX_BYTE USARTC0_DATA
    ISR(USARTC0_RXC_vect)
#endif

#if defined(__AVR_ATxmega16A4U__)
#else

{
    uint8_t byte = USART_RX_BYTE;
    
    switch (state)        
    {
        case 0:
            if (byte == 0xAA)
                state++;
            else
                state = 0;
            return;
        
        case 1:
            if (byte == 0xAF)
                state++;
            else
                state = 0;
            return;
            
        case 2:
            *((uint8_t*)(&sync_timestamp) + 0) = byte;
            state++;
            return;
            
        case 3:
            *((uint8_t*)(&sync_timestamp) + 1) = byte;
            state++;
            return;
            
        case 4:
            *((uint8_t*)(&sync_timestamp) + 2) = byte;
            state++;
            return;
            
        case 5:
            *((uint8_t*)(&sync_timestamp) + 3) = byte;
            state = 0;
           
			// We are here 98.2 +/-0.3 us after the last byte starts
			// This means that we are 672 - 98.2 = 573.8 before the second elapses
			// 573.8 us before the second elapses means that CCA should be equal to (1s - 573.8us) / 32us = 31232.1
			// Let's use CCA = 31232
		               
			// The last CCA written is 31233 = 0.999456 us, i.e., 544 us before the second elapses
		               
			// The 'device_lost_sync_counter' is incremented externally on every timer overflow, i.e., when each second elapses
			// If the sync is lost for more than 10 seconds,
		   
		    if (previous_timestamp + 1 == sync_timestamp)
			{
				if (device_lost_sync_counter >= 5)
				{
					// If the sync is lost for more than 5 seconds, restart the sync mechanism by converging
                
					converging = 12;	// 12*32us = 384 us away from the next 
					TCC1_CNT = 31231 - converging;
					TCC1_CCA = 31233;
				
					device_lost_sync_counter = 0;
				
				}
				else
				{				
					device_lost_sync_counter = 0;
				
					if (converging)
						converging -= 2;
							
					if (TCC1_CNT < TCC1_CCA)
					{
						TCC1_CNT = 31231 - converging;
					}
				}
            
			
				clock_was_just_updated_externaly = true;
            
				*core_timestamp_pointer = sync_timestamp;
				
				if (converging == 0)
					commonbank.R_HEARTBEAT |= B_IS_SYNCHRONIZED;
			}
			
			previous_timestamp = sync_timestamp;
            
            return;
    }
}

#endif