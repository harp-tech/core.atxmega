#include <avr/io.h>
#include "cpu.h"
#include "hwbp_sync.h"


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
        if ((*timestamp_B0 == 0xAA) && (*timestamp_B1 == 0xAF)) reti();
        if ((*timestamp_B1 == 0xAA) && (*timestamp_B2 == 0xAF)) reti();
        if ((*timestamp_B2 == 0xAA) && (*timestamp_B3 == 0xAF)) reti();
        
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
    static uint8_t converging = 7;
    uint8_t device_lost_sync_counter = 0;
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
    
    if (byte == 0xAA)
    {
        state = 0;
    }        
    else if (byte == 0xAF)
    {
        state = 1;
    }
    
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
            
            // CCA values:
            // 31217
            // 31233
            // (the average is 31225)
            
            if (device_lost_sync_counter >= 10)
                converging = 7;
            else
                device_lost_sync_counter = 0;
            
            /*
            TCC1_CTRLA = TC_CLKSEL_OFF_gc;
            TCC1_CTRLFSET = TC_CMD_RESET_gc;
            TCC1_PER = 31250;
            TCC1_INTCTRLA = INT_LEVEL_LOW;
            */
            
            TCC1_CNT = 31225 - converging;
            TCC1_CCA = 31233;
			
			clock_was_just_updated_externaly = true;
            
            /*
            TCC1_CTRLA = TIMER_PRESCALER_DIV1024;
            TCC1_INTCTRLB = INT_LEVEL_LOW;
            */
            
            if (converging)
                converging--;
            
            *core_timestamp_pointer = sync_timestamp;
            
            return;
    }
}

#endif