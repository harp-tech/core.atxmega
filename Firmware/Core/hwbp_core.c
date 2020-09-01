#include "cpu.h"
#include "hwbp_core.h"
#include "hwbp_core_regs.h"
#include "hwbp_core_types.h"
#include "hwbp_core_com.h"
#include "hwbp_sync.h"

/************************************************************************/
/* Note                                                                 */
/************************************************************************/
// - The Common Bank uses the timer TCC1.


/************************************************************************/
/* Define current version                                               */
/************************************************************************/
#define VH 1
#define VL 7
/************************************************************************/

/************************************************************************/
/* Define command headers                                               */
/************************************************************************/
#define CMD_READ				0x01
#define CMD_READ_OK			0x01
#define CMD_READ_ERROR		0x09
#define CMD_WRITE				0x02
#define CMD_WRITE_OK			0x02
#define CMD_WRITE_ERROR		0x0A
#define EVENT					0x03

/************************************************************************/
/* Allocate space for Events                                            */
/************************************************************************/
uint8_t reply_buff[MAX_PACKET_SIZE];

/************************************************************************/
/* Declaration of external variables for commands                       */
/************************************************************************/
extern uint8_t com_mode;
extern uint8_t rx_timeout;
extern bool rx_cmd_ready;
extern uint16_t cmd_len;

extern uint8_t rxbuff_hwbp_uart[];
#if HWBP_UART_RXBUFSIZ > 256
	extern uint16_t hwbp_uart_rx_pointer;
#else
	extern uint8_t hwbp_uart_rx_pointer;
#endif

extern uint8_t rxbuff_hwbp_usb[];
extern uint16_t hwbp_usb_rx_pointer;

/************************************************************************/
/* Declaration of external variables for events                         */
/************************************************************************/
extern uint8_t app_regs_type[];
extern uint16_t app_regs_n_elements[];
extern uint8_t *app_regs_pointer[];

/************************************************************************/
/* Control of send all the register                                     */
/************************************************************************/
static bool sending_registers = false;
static bool sending_com_registers = false;
static uint8_t sending_register_add;
static uint8_t sending_com_register_add;

/************************************************************************/
/* Control of send all the register                                     */
/************************************************************************/
static uint8_t default_device_name[25];
static bool enables_serial_number_update = false;

/************************************************************************/
/* Headers                                                              */
/************************************************************************/
static void parse_and_reply(uint8_t * packet_p, uint16_t len);
static void xmit_error(uint8_t header, uint8_t * packet);
static void xmit(uint8_t add, uint8_t header, bool use_core_timestamp);

/************************************************************************/
/* Common Regs Structure                                                */
/************************************************************************/
static struct CommonBank
{
	uint16_t	R_WHO_AM_I;
	uint8_t	R_HW_VERSION_H;
	uint8_t	R_HW_VERSION_L;
	uint8_t	R_ASSEMBLY_VERSION;
	uint8_t	R_HARP_VERSION_H;
	uint8_t	R_HARP_VERSION_L;
	uint8_t	R_FW_VERSION_H;
	uint8_t	R_FW_VERSION_L;
	uint32_t	R_TIMESTAMP_SECOND;
	uint16_t	R_TIMESTAMP_MICRO;
	uint8_t	R_OPERATION_CTRL;
	uint8_t	R_RESET_DEV;
	uint8_t R_DEVICE_NAME[25];
	uint16_t R_SERIAL_NUMBER;
} commonbank;

static uint8_t regs_type[] = {
	TYPE_U16,
	TYPE_U8,
	TYPE_U8,
	TYPE_U8,
	TYPE_U8,
	TYPE_U8,
	TYPE_U8,
	TYPE_U8,
	TYPE_U32,
	TYPE_U16,
	TYPE_U8,
	TYPE_U8,
    TYPE_U8,
	TYPE_U16
};

static uint16_t regs_n_elements[] = {
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	25,
	1
};

static uint8_t *regs_pointer[] = {
	(uint8_t*)(&commonbank.R_WHO_AM_I),
	(uint8_t*)(&commonbank.R_HW_VERSION_H),
	(uint8_t*)(&commonbank.R_HW_VERSION_L),
	(uint8_t*)(&commonbank.R_ASSEMBLY_VERSION),
	(uint8_t*)(&commonbank.R_HARP_VERSION_H),
	(uint8_t*)(&commonbank.R_HARP_VERSION_L),
	(uint8_t*)(&commonbank.R_FW_VERSION_H),
	(uint8_t*)(&commonbank.R_FW_VERSION_L),
	(uint8_t*)(&commonbank.R_TIMESTAMP_SECOND),
	(uint8_t*)(&commonbank.R_TIMESTAMP_MICRO),
	(uint8_t*)(&commonbank.R_OPERATION_CTRL),
	(uint8_t*)(&commonbank.R_RESET_DEV),
    (uint8_t*)(commonbank.R_DEVICE_NAME),
    (uint8_t*)(&commonbank.R_SERIAL_NUMBER)
};



/************************************************************************/
/* Variables and definitions                                            */
/************************************************************************/
static uint16_t core_app_mem_size_to_save;
static uint8_t * core_pointer_to_app_regs;
static uint8_t core_num_of_app_registers;

#if defined(__AVR_ATxmega16A4U__)
    #define SET_STATE_LED PORTD_OUTSET = (1<<5)
    #define CLR_STATE_LED PORTD_OUTCLR = (1<<5)
    #define TGL_STATE_LED PORTD_OUTTGL = (1<<5)
    #define CONFIG_STATE_LED io_pin2out(&PORTD, 5, OUT_IO_DIGITAL, IN_EN_IO_DIS);
#endif
#if defined(__AVR_ATxmega32A4U__)
	#define SET_STATE_LED PORTR_OUTSET = (1<<0)
	#define CLR_STATE_LED PORTR_OUTCLR = (1<<0)
	#define TGL_STATE_LED PORTR_OUTTGL = (1<<0)
	#define CONFIG_STATE_LED io_pin2out(&PORTR, 0, OUT_IO_DIGITAL, IN_EN_IO_DIS);
#endif
#if defined(__AVR_ATxmega64A4U__)
	#define SET_STATE_LED PORTD_OUTSET = (1<<5)
	#define CLR_STATE_LED PORTD_OUTCLR = (1<<5)
	#define TGL_STATE_LED PORTD_OUTTGL = (1<<5)
	#define CONFIG_STATE_LED io_pin2out(&PORTD, 5, OUT_IO_DIGITAL, IN_EN_IO_DIS);
#endif
#if defined(__AVR_ATxmega128A4U__)
	#define SET_STATE_LED PORTR_OUTSET = (1<<0)
	#define CLR_STATE_LED PORTR_OUTCLR = (1<<0)
	#define TGL_STATE_LED PORTR_OUTTGL = (1<<0)
	#define CONFIG_STATE_LED io_pin2out(&PORTR, 0, OUT_IO_DIGITAL, IN_EN_IO_DIS);
#endif
#if defined(__AVR_ATxmega128A1U__)
	#define SET_STATE_LED PORTA_OUTSET = (1<<6)
	#define CLR_STATE_LED PORTA_OUTCLR = (1<<6)
	#define TGL_STATE_LED PORTA_OUTTGL = (1<<6)
	#define CONFIG_STATE_LED io_pin2out(&PORTA, 6, OUT_IO_DIGITAL, IN_EN_IO_DIS);
#endif

/************************************************************************/
/* Initialize Common Bank and used peripherals                          */
/************************************************************************/
#define F_CPU 32000000
static uint8_t _500us_cca_values[] = {15, 16, 16, 15, 16, 15, 16, 16};
static uint8_t _500us_cca_index = 0;

#define EEPROM_ADD_MEM_IN_USE       0
#define EEPROM_ADD_R_OPERATION_CTRL 1
#define EEPROM_ADD_R_FW_VERSION_H   2
#define EEPROM_ADD_R_HARP_VERSION_H 3
#define EEPROM_ADD_SN_HIGH          4
#define EEPROM_ADD_SN_LOW           5
#define EEPROM_ADD_AVAILABLE2       6
#define EEPROM_ADD_R_DEVICE_NAME    7
#define EEPROM_ADD_APP_REG          32

#define R_DEVICE_NAME_SIZE          25

#define EEPROM_RST_USE_REGS_DEF     0xDF
#define EEPROM_RST_USE_REGS_EE      0xEE

static void write_device_name_to_eeprom(const uint8_t *dev_name)
{
    bool end_of_line_detected = false;
        
    for (uint8_t i = 0; i < R_DEVICE_NAME_SIZE; i++)
    {
        /* Check if there is an end of line */
        if (dev_name[i] == 0)
        {
            end_of_line_detected = true;
        }
            
        if (end_of_line_detected)
        {
            /* If end of line was detected, the rest of the register will be zeros */
            eeprom_wr_byte(EEPROM_ADD_R_DEVICE_NAME + i, 0);
        }
        else
        {
            eeprom_wr_byte(EEPROM_ADD_R_DEVICE_NAME + i, dev_name[i]);
        }
    }        
}

static void read_device_name_to_register(void)
{
    for (uint8_t i = 0; i < R_DEVICE_NAME_SIZE; i++)
    {
        commonbank.R_DEVICE_NAME[i] = eeprom_rd_byte(EEPROM_ADD_R_DEVICE_NAME + i);
    }
}

void core_func_start_core (
    const uint16_t who_am_i,
    const uint8_t hwH,
    const uint8_t hwL,
    const uint8_t fwH,
    const uint8_t fwL,
    const uint8_t assembly,
    uint8_t *pointer_to_app_regs,
    const uint16_t app_mem_size_to_save,
    const uint8_t num_of_app_registers,
    const uint8_t *device_name)
{	
	/* Shut down watchdog */
	wdt_disable();
	
	/* Start external clock */
	cpu_config_clock(F_CPU, true, true);
	
	/* Configure sleep mode */
	SLEEP_CTRL = SLEEP_SMODE_IDLE_gc | SLEEP_SEN_bm;
	
	/* Initialize common register bank */
	commonbank.R_WHO_AM_I = who_am_i;	
	commonbank.R_HW_VERSION_H = hwH;
	commonbank.R_HW_VERSION_L = hwL;
	commonbank.R_ASSEMBLY_VERSION = assembly;
	commonbank.R_HARP_VERSION_H = VH;
	commonbank.R_HARP_VERSION_L = VL;
	commonbank.R_FW_VERSION_H = fwH;
	commonbank.R_FW_VERSION_L = fwL;
	commonbank.R_TIMESTAMP_SECOND = 0;		// Timestamps starts from 0 (ZERO)
	commonbank.R_TIMESTAMP_MICRO = 0;		// Timestamps starts from 0 (ZERO)	
	//commonbank.R_RESET_DEV = 0;
        
    /* Read versions from EEPROM */
    uint8_t previousFwH   = eeprom_rd_byte(EEPROM_ADD_R_FW_VERSION_H);
    uint8_t previousHarpH = eeprom_rd_byte(EEPROM_ADD_R_HARP_VERSION_H);
    
    /* Check if the versions on EEPROM were never updated */
    if (previousHarpH == 0xFF)
    {
        eeprom_wr_byte(EEPROM_ADD_R_FW_VERSION_H, fwH);
        eeprom_wr_byte(EEPROM_ADD_R_HARP_VERSION_H, VH);
        
        previousFwH   = eeprom_rd_byte(EEPROM_ADD_R_FW_VERSION_H);
        previousHarpH = eeprom_rd_byte(EEPROM_ADD_R_HARP_VERSION_H);
        
        /* Write the device name for the first time */
        write_device_name_to_eeprom(device_name);        
        
        /* Force device to start with default values */
        eeprom_wr_byte(EEPROM_ADD_MEM_IN_USE, EEPROM_RST_USE_REGS_DEF);
    }
    
    /* Check if the device was updated to a higher major version */
    //if ((fwH != previousFwH) || ((VH != previousHarpH))
    if ((fwH != previousFwH) || (VH != previousHarpH))   // Don't need to reset registers on this major version (VH = 1)
    {
        /* Force device to start with default values */
        eeprom_wr_byte(EEPROM_ADD_MEM_IN_USE, EEPROM_RST_USE_REGS_DEF);
        
        /* Update EERPOM */
        eeprom_wr_byte(EEPROM_ADD_R_FW_VERSION_H, fwH);
        eeprom_wr_byte(EEPROM_ADD_R_HARP_VERSION_H, VH);
    }        
	
	/* Callback to start the device */
	core_callback_1st_config_hw_after_boot();

	/* Check if the EEPROM is clean or have data */
	if (eeprom_rd_byte(EEPROM_ADD_MEM_IN_USE) == EEPROM_RST_USE_REGS_EE)
	{
		commonbank.R_OPERATION_CTRL = eeprom_rd_byte(EEPROM_ADD_R_OPERATION_CTRL);

		for (uint16_t i = 0; i < app_mem_size_to_save; i++)
			*(pointer_to_app_regs + i) = eeprom_rd_byte(EEPROM_ADD_APP_REG + i);
		
		commonbank.R_RESET_DEV = B_BOOT_EE;
		core_callback_registers_were_reinitialized();
	}
	else
	{
		commonbank.R_OPERATION_CTRL = B_OPLEDEN | B_VISUALEN | GM_OP_MODE_STANDBY;

		commonbank.R_RESET_DEV = B_BOOT_DEF;
		core_callback_reset_registers();
		core_callback_registers_were_reinitialized();
	}
	core_app_mem_size_to_save = app_mem_size_to_save;
	core_pointer_to_app_regs = pointer_to_app_regs;
	core_num_of_app_registers = num_of_app_registers;
    
    /* Check if the device's name was never updated before */
    if (eeprom_rd_byte(EEPROM_ADD_R_DEVICE_NAME) == 0xFF)
    {
        write_device_name_to_eeprom(device_name);
    }
    
    /* Update device's name register */
    read_device_name_to_register();
    
    /* Get device name for further updates */
    for (uint8_t i = 0; i < R_DEVICE_NAME_SIZE; i++)
    {
        *(default_device_name + i) = commonbank.R_DEVICE_NAME[i];
    }
	
	/* Get serial number */
	commonbank.R_SERIAL_NUMBER = eeprom_rd_byte(EEPROM_ADD_SN_HIGH);
	commonbank.R_SERIAL_NUMBER = ((commonbank.R_SERIAL_NUMBER << 8) & 0xFF00) | eeprom_rd_byte(EEPROM_ADD_SN_LOW);

	/* Start 1 second timer */
	timer_type1_enable(&TCC1, TIMER_PRESCALER_DIV1024, 31250, INT_LEVEL_LOW);

	/* Re-start 500 us timer */
	TCC1_CCA = _500us_cca_values[_500us_cca_index++] - 1;
	TCC1_INTCTRLB = INT_LEVEL_LOW;

	/* Configure PORT to drive the op LED */
	CONFIG_STATE_LED;
	SET_STATE_LED;

	/* Configure main UART with 1 Mbit/s */
	hwbp_com_uart_init(1, 0, false);
	hwbp_com_uart_enable();
    
    /* Configure timestamp UART */    
    initialize_timestamp_uart(&commonbank.R_TIMESTAMP_SECOND);
}

void core_func_catastrophic_error_detected(void)
{
	core_callback_catastrophic_error_detected();

	while(1)
	{
		for (uint32_t i = 0; i < F_CPU/100; i++)
			;

		TGL_STATE_LED;
	}
}

/************************************************************************/
/* One second timer                                                     */
/************************************************************************/
static uint8_t shutdown_counter = 0;

extern uint8_t device_lost_sync_counter;

ISR(TCC1_OVF_vect, ISR_NAKED)
{
	commonbank.R_TIMESTAMP_SECOND++;
   
    RESET_TIMESTAMP_COUNTER;
    INCREASE_LOST_SYNC_COUNTER;

	/* Re-start 500 us timer */	
	TCC1_CCA = _500us_cca_values[0] - 1;
	_500us_cca_index = 1;
 	
    core_callback_t_before_exec();    
	
	TCC1_INTFLAGS |= TC1_CCAIF_bm;
	
	core_callback_t_new_second();
	core_callback_t_1ms();	
	
	if (commonbank.R_OPERATION_CTRL & B_ALIVE_EN)
        core_func_send_event(ADD_R_TIMESTAMP_SECOND, true);
	
	uint8_t op_led_comp;
	switch (commonbank.R_OPERATION_CTRL & MSK_OP_MODE)
	{
		case GM_OP_MODE_STANDBY:
			op_led_comp = 4;
			break;

		case GM_OP_MODE_ACTIVE:
			op_led_comp = 2;
			break;

		default:	op_led_comp = 1;
	};
		
	if (*((uint8_t*)(&commonbank.R_TIMESTAMP_SECOND)) & op_led_comp)
	{
		CLR_STATE_LED;
	}
	else
	{
		if (commonbank.R_OPERATION_CTRL & B_OPLEDEN)
		{
			SET_STATE_LED;
		}
	}

	core_callback_t_after_exec();

	if (TCC1_INTFLAGS & TC1_CCAIF_bm)
		if (shutdown_counter == 0)
			core_func_catastrophic_error_detected();

	reti();
}


/************************************************************************/
/* 500 us timer used to call app                                        */
/************************************************************************/
uint16_t CTS_timeout_counter = 0;
#define CTS_TIMEOUT_MS 3000

#if HWBP_UART_TXBUFSIZ > 256
    extern uint16_t hwbp_uart_tail;
    extern uint16_t hwbp_uart_head;
#else
    extern uint8_t hwbp_uart_tail;
    extern uint8_t hwbp_uart_head;
#endif

ISR(TCC1_CCA_vect, ISR_NAKED)
{   
	SYNC_TRIGGER_TIMER;
    
    core_callback_t_before_exec();

	TCC1_CCA += _500us_cca_values[_500us_cca_index++ & 0x07];	

	//core_send_event(ADD_R_TIMESTAMP_SECOND);

	if ((_500us_cca_index & 1) == 0)
	{
		/* Make sure that any UART or USB interrupts will not
		update rx buffers */
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
		
		/* Check timeout and, if happens, reset pointers  */
		if (rx_timeout > 0)
		{
			if (--rx_timeout == 0)
			{
				rx_timeout = 0;
				rx_cmd_ready = false;

				hwbp_uart_rx_pointer = 0;
				//hwbp_usb_rx_pointer = 0;
			}
		}		

		/* Check if a command is ready to execute */
		if (rx_cmd_ready)
		{
			parse_and_reply(rxbuff_hwbp_uart, cmd_len);

			rx_cmd_ready = false;
		}
        
        
        if (com_mode == COM_MODE_UART)
        {
            /* If CTS not ok, destroy data and go to Standby if in Active Mode */
            if (read_io(HWBP_UART_CTS_PORT, HWBP_UART_CTS_pin))
            {
                if (++CTS_timeout_counter == CTS_TIMEOUT_MS)
                {
                    CTS_timeout_counter = 0;
                    
                    if ((commonbank.R_OPERATION_CTRL & MSK_OP_MODE) == GM_OP_MODE_ACTIVE )
                    {
                        /* Return to Standby Mode */
                        uint8_t aux_op_ctrl = commonbank.R_OPERATION_CTRL;
                        
                        aux_op_ctrl &= ~MSK_OP_MODE;
                        aux_op_ctrl |= GM_OP_MODE_STANDBY;
                        hwbp_write_common_reg(ADD_R_OPERATION_CTRL, TYPE_U8, &aux_op_ctrl, 1);
                    }
                    
                    /* Destroy all data on the buffers */
                    hwbp_uart_tail = 0;
                    hwbp_uart_head = 0;
                
                    /* Disable uart DRE interrupt until RTS is logic low */
                    HWBP_UART_UART.CTRLA &= ~(USART_DREINTLVL_OFF_gc | USART_DREINTLVL_gm);
                }                
            }
            else
            {
                CTS_timeout_counter = 0;
            }
        }            
		
		if (com_mode == COM_MODE_UART)
		{
			/* Disable high level interrupts */
			PMIC_CTRL = PMIC_RREN_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;

			/* Enable USART RX */
			HWBP_UART_UART.CTRLA |= (HWBP_UART_RX_INT_LEVEL<< 4);

			/* Re-enable high level interrupts */
			PMIC_CTRL = PMIC_RREN_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;

			/* Enable rx external hardware */
			enable_hwbp_uart_rx;
		}

		core_callback_t_500us();
	}
	else
	{
		core_callback_t_1ms();
	}

	if (shutdown_counter > 0)
		if (--shutdown_counter == 0)			
			wdt_reset_device();

	/* Send common registers */
	if (sending_com_registers)
	{	
		/* Each 2 milliseconds */
		if ((_500us_cca_index & 0x03) == 2)
		{
			if (hwbp_read_common_reg(sending_com_register_add, regs_type[sending_com_register_add]))
				xmit(sending_com_register_add, CMD_READ_OK, true);

			if (++sending_com_register_add == COMMON_BANK_ABSOLUTE_ADD_MAX )
				sending_com_registers = false;
		}
	}

	/* Send app registers */
	if (sending_registers)
	{
		/* Each 2 milliseconds */
		if ((_500us_cca_index & 0x03) == 0)
		{
			if (core_read_app_register(sending_register_add + 0x20, app_regs_type[sending_register_add]))
				xmit(sending_register_add + 0x20, CMD_READ_OK, true);

			if (++sending_register_add == core_num_of_app_registers )
				sending_registers = false;
		}
	}

	core_callback_t_after_exec();

	if (TCC1_INTFLAGS & TC1_CCAIF_bm)
		if (shutdown_counter == 0)
			core_func_catastrophic_error_detected();

	reti();
}


/************************************************************************/
/* Parse received command                                               */
/************************************************************************/
#define check_checksum(array, leng, checksum_) checksum_ = 0; for (uint8_t iii = 0; iii < leng; iii++) checksum_ += array[iii];

static void parse_and_reply(uint8_t * packet_p, uint16_t len)
{
	uint8_t add = (*(packet_p+1) != 255) ? *(packet_p+2) : *(packet_p+4);			// add
	uint8_t port = (*(packet_p+1) != 255) ? *(packet_p+3) : *(packet_p+5);			// port
	uint8_t type = (*(packet_p+1) != 255) ? *(packet_p+4) : *(packet_p+6);			// type
	uint8_t * payload_p = (*(packet_p+1) != 255) ? (packet_p+5) : (packet_p+7);	// payload pointer
	uint16_t payload_n_elements = (len - 4) / (type & MSK_TYPE_LEN);					// number of elements on the payload array
	
	bool ok = false;
	
	/* Check checksum */
	uint8_t checksum;
	check_checksum(packet_p, cmd_len+1, checksum);
	if (packet_p[cmd_len+1] != checksum)
		return;

	/* Check port */
	if (port == 0xFF)	// This device
	{
		/* Read registers */
		if (*packet_p == CMD_READ)
		{
			if (add <= COMMON_BANK_ABSOLUTE_ADD_MAX)
            {
				ok = hwbp_read_common_reg(add, type);
            }                
			else
            {
				ok = core_read_app_register(add, type);
            }                
            
            if (!(commonbank.R_OPERATION_CTRL & B_MUTE_RPL))
            {
			    if (ok)
                {
				    xmit(add, CMD_READ_OK, true);
                }                    
			    else
                {
				    xmit_error(CMD_READ_ERROR, packet_p);
                }                    
            }                    
		}

		/* Write registers */
		else
		{
			if (add <= COMMON_BANK_ABSOLUTE_ADD_MAX)
            {
				ok = hwbp_write_common_reg(add, type, payload_p, payload_n_elements);
            }                
			else
            {
				ok = core_write_app_register(add, type, payload_p, payload_n_elements);
            }                
			
            if (!(commonbank.R_OPERATION_CTRL & B_MUTE_RPL))
            {
			    if (ok)
                {
				    xmit(add, CMD_WRITE_OK, true);
                }                    
			    else
                {
				    xmit_error(CMD_WRITE_ERROR, packet_p);
                }                    
            }                    
		}
	}
}

/************************************************************************/
/* Send event                                                           */
/************************************************************************/
static uint8_t * reg_p;

static void xmit_error(uint8_t header, uint8_t * packet)
{	
	/* Check if packet is above limits */
	if ( (packet[1] + 2 + 6) > (MAX_PACKET_SIZE - 1) )
		return;
	
	/* Shift payload by 6 positions */
	for (uint8_t i = 0; i < packet[1] - 3; i++)
		packet[11 + i] = packet[5 + i];
	
	/* Update values */
	packet[0] = header;
	packet[1] += 6;													// update len
	packet[4] |= MSK_TIMESTAMP_AT_PAYLOAD;							// update type

	/* Update second */
	if (TCC1_INTFLAGS & TC1_OVFIF_bm)
	{
		/* Add 1 if the new second interrupt FLAG is already set */
		*((uint32_t*)(packet+5)) = commonbank.R_TIMESTAMP_SECOND+1;	// update second
        *((uint16_t*)(packet+9)) = TCC1_CNT;				        // update microsecond
	}
	else
	{
		*((uint32_t*)(packet+5)) = commonbank.R_TIMESTAMP_SECOND;	// update second
        *((uint16_t*)(packet+9)) = TCC1_CNT;						// update microsecond
        
        if (_500us_cca_index == 208)
        {
            /* It may be on the last timer iteration */
            
            if (*((uint16_t*)(packet+9)) < _500us_cca_values[0])
            {
                /* This is not possible unless the timer overflow happened on this instant */
                
                *((uint32_t*)(packet+5)) = commonbank.R_TIMESTAMP_SECOND+1; // update second
            }
        }            
	}
	
	check_checksum(packet, packet[1] + 1, packet[packet[1] + 1]);	// update checksum

	/* Send Error */
	if (com_mode == COM_MODE_UART)
	{
		/* Disable TX interrupts before update tx buffers */
		PMIC_CTRL = PMIC_RREN_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;			
		
		/* Disable USART TX */
		uint8_t TxInterruptState = HWBP_UART_UART.CTRLA & USART_DREINTLVL_gm;
		HWBP_UART_UART.CTRLA &= ~(USART_DREINTLVL_gm);

		/* Xmit trough USART */
		hwbp_uart_xmit(packet, packet[1] + 2);

		/* Recover USART TX Interrupt state */
		HWBP_UART_UART.CTRLA |= TxInterruptState;

		/* Re-enable high level interrupts */
		PMIC_CTRL = PMIC_RREN_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	}
	else
	{

	}
}

static uint32_t user_R_TIMESTAMP_SECOND = 0;
static uint16_t user_TCC1_CNT = 0;

void core_func_update_user_timestamp(uint32_t seconds, uint16_t useconds)
{
	user_R_TIMESTAMP_SECOND = seconds;
	user_TCC1_CNT = useconds;
}

void core_func_read_user_timestamp(uint32_t *seconds, uint16_t *useconds)
{
    *seconds = user_R_TIMESTAMP_SECOND;
    *useconds = user_TCC1_CNT;
}

void core_func_mark_user_timestamp(void)
{
    if (TCC1_INTFLAGS & TC1_OVFIF_bm)
    {
        /* Add 1 if the new second interrupt FLAG is already set */
        user_R_TIMESTAMP_SECOND = commonbank.R_TIMESTAMP_SECOND+1;
        user_TCC1_CNT = TCC1_CNT;
    }
    else
    {
        user_R_TIMESTAMP_SECOND = commonbank.R_TIMESTAMP_SECOND;
        user_TCC1_CNT = TCC1_CNT;
        	
        if (_500us_cca_index == 208)
        {
            /* It may be on the last timer iteration */
            	
            if (user_TCC1_CNT < _500us_cca_values[0])
            {
                /* This is not possible unless the timer overflow happened on this instant */
                	
                user_R_TIMESTAMP_SECOND = commonbank.R_TIMESTAMP_SECOND+1; // update second
            }
        }
    }
}

static void xmit(uint8_t add, uint8_t header, bool use_core_timestamp)
{
	uint8_t type;
	uint16_t n_elements;

	/* Get type and number of elements */
	if (add >= 0x20)
	{
		type = app_regs_type[add-0x20];
		n_elements = app_regs_n_elements[add-0x20];
		reg_p = app_regs_pointer[add-0x20];
	}
	else if (add <= COMMON_BANK_ADD_MAX)
	{
		type = regs_type[add];
		n_elements = regs_n_elements[add];
		reg_p = regs_pointer[add];
	}
	else
		return;
	
	/* Update Event's temporary buffer */
	reply_buff[0] = header;	
	reply_buff[1] = (type & MSK_TYPE_LEN) * (n_elements) + 10;			    // update len
	reply_buff[2] = add;													// update add
	reply_buff[3] = 255;													// update port	
	reply_buff[4] = type | MSK_TIMESTAMP_AT_PAYLOAD;						// update type
	
	if (use_core_timestamp)
	{
	    if (TCC1_INTFLAGS & TC1_OVFIF_bm)
	    {
    	    /* Add 1 if the new second interrupt FLAG is already set */
    	    *((uint32_t*)(reply_buff+5)) = commonbank.R_TIMESTAMP_SECOND+1;	// update second
    	    *((uint16_t*)(reply_buff+9)) = TCC1_CNT;				        // update microsecond
	    }
	    else
	    {
    	    *((uint32_t*)(reply_buff+5)) = commonbank.R_TIMESTAMP_SECOND;	// update second
    	    *((uint16_t*)(reply_buff+9)) = TCC1_CNT;						// update microsecond
    	
    	    if (_500us_cca_index == 208)
    	    {
        	    /* It may be on the last timer iteration */
        	
        	    if (*((uint16_t*)(reply_buff+9)) < _500us_cca_values[0])
        	    {
            	    /* This is not possible unless the timer overflow happened on this instant */
            	
            	    *((uint32_t*)(reply_buff+5)) = commonbank.R_TIMESTAMP_SECOND+1; // update second
        	    }
    	    }
	    }
        
        if (add == ADD_R_TIMESTAMP_SECOND)
        {
            if (header == EVENT)
            {
                *((uint16_t*)(reply_buff+9)) = 0;                           // Second's Event always have the us equal to 0
            }
        }            
	}
	else
	{
		*((uint32_t*)(reply_buff+5)) = user_R_TIMESTAMP_SECOND;				// update second
		*((uint16_t*)(reply_buff+9)) = user_TCC1_CNT;						// update microsecond
	}

	/* Check if packet is above limits */
	if ( (reply_buff[1] + 2) > (MAX_PACKET_SIZE - 1) )
		return;

	/* Read the register */
	uint8_t len = (type & MSK_TYPE_LEN) * n_elements;
	uint8_t index = 0;
	
	do
	{
		*(reply_buff+11+index) = *(reg_p+index);
		index++;
	} while(--len != 0);

	/* Calculate checksum */
	len = reply_buff[1] + 2;
	uint8_t checksum;
	check_checksum(reply_buff, len-1, checksum);
	reply_buff[len-1] = checksum;													// update checksum


	/* Send Event */
	if (com_mode == COM_MODE_UART)
	{
		/* Disable TX interrupts before update tx buffers */
		PMIC_CTRL = PMIC_RREN_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;
			
		/* Disable USART TX */
		uint8_t TxInterruptState = HWBP_UART_UART.CTRLA & USART_DREINTLVL_gm;
		HWBP_UART_UART.CTRLA &= ~(USART_DREINTLVL_gm);

		/* Xmit trough USART */
		hwbp_uart_xmit(reply_buff, reply_buff[1] + 2);
		
		/* Re-enable USART TX */
		HWBP_UART_UART.CTRLA |= TxInterruptState;

		/* Re-enable high level interrupts */
		PMIC_CTRL = PMIC_RREN_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;

	}
	else
	{

	}
}

void core_func_send_event(uint8_t add, bool use_core_timestamp)
{
	if ((commonbank.R_OPERATION_CTRL & MSK_OP_MODE) == GM_OP_MODE_ACTIVE)
		xmit(add, EVENT, use_core_timestamp);
}

/************************************************************************/
/* Read common bank                                                     */
/************************************************************************/
bool hwbp_read_common_reg(uint8_t add, uint8_t type)
{
	/* Check if it will not access forbidden memory */
	if (add > COMMON_BANK_ADD_MAX)
		return false;
	
	/* Check if type matches */
	if (regs_type[add] != type)
		return false;

	/* Check if there is something to do before reading */
	/*
	switch (add)
	{
		case ADD_R_TIMESTAMP_SECOND:
			break;
			
		case ADD_R_TIMESTAMP_MICRO:
			* Update R_TIMESTAMP_MICRO *
			commonbank.R_TIMESTAMP_MICRO = TCC1_CNT;
			break;
	}
	*/
	
	/* Update R_TIMESTAMP_MICRO */
	if (add == ADD_R_TIMESTAMP_MICRO)
		commonbank.R_TIMESTAMP_MICRO = TCC1_CNT;

	/* Return success */
	return true;
}

/************************************************************************/
/* Write to common bank                                                 */
/************************************************************************/
bool hwbp_write_common_reg(uint8_t add, uint8_t type, uint8_t * content, uint16_t n_elements)
{
	/* Check if it will not access forbidden memory */
	if (add > COMMON_BANK_ADD_MAX)
		return false;
		
	/* Check if type matches */
	if (regs_type[add] != type)
		return false;

	/* Check if the number of elements matches */
	if (regs_n_elements[add] != n_elements)
		return false;
	
	/* R_TIMESTAMP_SECOND */
	if (add == ADD_R_TIMESTAMP_SECOND)
	{
		/* Update register */
		commonbank.R_TIMESTAMP_SECOND = *((uint32_t*)(content));

		/* Return success */
		return true;
	}

	/* R_OPERATION_CTRL */
	else if (add == ADD_R_OPERATION_CTRL)
	{
		/* Disable of the OP LED? */
		if (!(*content & B_OPLEDEN))
		{
			CLR_STATE_LED;
		}
		
		/* Save previous configuration in a local variable */
		uint8_t temp_R_OPERATION_CTRL = commonbank.R_OPERATION_CTRL;
		
		/* Update register */
		commonbank.R_OPERATION_CTRL = *content & (B_ALIVE_EN | B_OPLEDEN | B_VISUALEN | B_MUTE_RPL | MSK_OP_MODE);
		
		/* Verify if a transition occurs on the B_VISUALEN */
		if ((*content & B_VISUALEN) && !(temp_R_OPERATION_CTRL & B_VISUALEN))
			core_callback_visualen_to_on();
		if (!(*content & B_VISUALEN) && (temp_R_OPERATION_CTRL & B_VISUALEN))
			core_callback_visualen_to_off();

		if (*content & B_DUMP)
		{
			/* Enable the dispatch of the app registers */
			sending_registers = true;
			sending_register_add = 0;			
			sending_com_registers = true;
			sending_com_register_add = 0;

			/* Clear DUMP bit */
			commonbank.R_OPERATION_CTRL &= (~B_DUMP);
		}

		/* Check if the Mode changed */
		if (((*content) & MSK_OP_MODE) == GM_OP_MODE_STANDBY && (temp_R_OPERATION_CTRL & MSK_OP_MODE) != GM_OP_MODE_STANDBY )
		{
			sending_registers = false;

			core_callback_device_to_standby();
		}
		if (((*content) & MSK_OP_MODE) == GM_OP_MODE_ACTIVE && (temp_R_OPERATION_CTRL & MSK_OP_MODE) != GM_OP_MODE_ACTIVE )
		{	
			core_callback_device_to_active();
		}
		if (((*content) & MSK_OP_MODE) == GM_OP_MODE_SPEED && (temp_R_OPERATION_CTRL & MSK_OP_MODE) != GM_OP_MODE_SPEED )
		{			
			core_callback_device_to_speed();			
		}
		
		/* Return success */
		return true;
	}

	/* ADD_R_RESET_APP */
	else if (add == ADD_R_RESET_DEV)
	{		
		/* Perform a reset to default values*/		
		if (*content == B_RST_DEF)
		{            
            write_device_name_to_eeprom(default_device_name);
            
			eeprom_wr_byte(EEPROM_ADD_MEM_IN_USE, EEPROM_RST_USE_REGS_DEF);
            
			shutdown_counter = 3;
		}

		/* Perform a reset to EEPROM values*/		
		if (*content == B_RST_EE)
		{
			/* Only if the device is booting from EEPROM */
			if (eeprom_rd_byte(EEPROM_ADD_MEM_IN_USE) == EEPROM_RST_USE_REGS_EE)
				shutdown_counter = 3;
			else
				return false;
		}
		
		/* Save register and reset device */
		if (*content == B_SAVE)
		{
			eeprom_wr_byte(EEPROM_ADD_MEM_IN_USE, EEPROM_RST_USE_REGS_EE);
			eeprom_wr_byte(EEPROM_ADD_R_OPERATION_CTRL, commonbank.R_OPERATION_CTRL);

			for (uint16_t i = 0; i < core_app_mem_size_to_save; i++)
				eeprom_wr_byte(EEPROM_ADD_APP_REG + i, *(core_pointer_to_app_regs + i));

			shutdown_counter = 3;
		}
        
        /* Update device's name to default */
        if (*content == B_NAME_TO_DEFAULT)
        {
            eeprom_wr_byte(EEPROM_ADD_R_DEVICE_NAME, 0xFF);

            shutdown_counter = 3;
        }

		/* Return success */
		return true;
	}
    
    /* ADD_R_DEVICE_NAME */
    else if (add == ADD_R_DEVICE_NAME)
    {        
        write_device_name_to_eeprom(content);
        
        shutdown_counter = 3;
        
        /* Return success */
        return true;
    }
	
	/* ADD_R_DEVICE_NAME */
	else if (add == ADD_R_SERIAL_NUMBER)
	{
		if (*((uint16_t*)(content)) == 0xFFFF)
		{
			enables_serial_number_update = true;
		}
		else
		{
			if (enables_serial_number_update)
			{
				//uint16_t serial_number = *((uint16_t*)(content));
				eeprom_wr_byte(EEPROM_ADD_SN_LOW,  *(content+0));
				eeprom_wr_byte(EEPROM_ADD_SN_HIGH, *(content+1));
				
				shutdown_counter = 3;				
			}
			else
			{
				/* Not possible to write a serial number if not enabled before */
				return false;
			}
		}
		
		/* Return success */
		return true;
	}

	/* Return error */
	return false;
}


/************************************************************************/
/*  Read R_TIMESTAMP_SECOND and R_TIMESTAMP_MICRO                       */
/************************************************************************/
uint32_t core_func_read_R_TIMESTAMP_SECOND(void)
{	
	return commonbank.R_TIMESTAMP_SECOND;
}

uint16_t core_func_read_R_TIMESTAMP_MICRO(void)
{
	return TCC1_CNT;
}


/************************************************************************/
/* Change from Speed Mode to Standby Mode                               */
/************************************************************************/
void core_func_leave_speed_mode_and_go_to_standby_mode(void)
{
	if ((commonbank.R_OPERATION_CTRL & MSK_OP_MODE) == GM_OP_MODE_SPEED)
	{
		commonbank.R_OPERATION_CTRL = (commonbank.R_OPERATION_CTRL & (~MSK_OP_MODE)) | GM_OP_MODE_STANDBY;
		core_callback_device_to_standby();
	}
}


/************************************************************************/
/* Boolean functions                                                    */
/************************************************************************/
bool core_bool_is_visual_enabled(void)
{
	return (commonbank.R_OPERATION_CTRL & B_VISUALEN) ? true : false;
}
bool core_bool_speed_mode_is_in_use(void)
{
	return ((commonbank.R_OPERATION_CTRL & MSK_OP_MODE) == GM_OP_MODE_SPEED) ? true : false;
}