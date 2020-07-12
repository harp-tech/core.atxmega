#ifndef _APP_IOS_AND_REGS_H_
#define _APP_IOS_AND_REGS_H_
#include "cpu.h"

void init_ios(void);
/************************************************************************/
/* Definition of input pins                                             */
/************************************************************************/
// INPUT                  Description: Digital input 0

#define read_INPUT read_io(PORTB, 1)            // INPUT

/************************************************************************/
/* Definition of output pins                                            */
/************************************************************************/
// OUTPUT                 Description: Digital output 0

/* OUTPUT */
#define set_OUTPUT set_io(PORTB, 0)
#define clr_OUTPUT clear_io(PORTB, 0)
#define tgl_OUTPUT toggle_io(PORTB, 0)
#define read_OUTPUT read_io(PORTB, 0)


/************************************************************************/
/* Registers' structure                                                 */
/************************************************************************/
typedef struct
{
	uint8_t REG_DUMMY0;
	int16_t REG_DUMMY1[16];
} AppRegs;

/************************************************************************/
/* Registers' address                                                   */
/************************************************************************/
/* Registers */
#define ADD_REG_DUMMY0                      32 // U8     A register without any usefull description
#define ADD_REG_DUMMY1                      33 // I16    A register without any usefull description

/************************************************************************/
/* PWM Generator registers' memory limits                               */
/*                                                                      */
/* DON'T change the APP_REGS_ADD_MIN value !!!                          */
/* DON'T change these names !!!                                         */
/************************************************************************/
/* Memory limits */
#define APP_REGS_ADD_MIN                    0x20
#define APP_REGS_ADD_MAX                    0x21
#define APP_NBYTES_OF_REG_BANK              33

/************************************************************************/
/* Registers' bits                                                      */
/************************************************************************/
#define B_DI0                              (1<<0)       // Digital input 0
#define B_DI1                              (1<<1)       // Digital input 1
#define B_DI2                              (1<<2)       // Digital input 2
#define MSK_ADC_SAMPLE_RATE                0x03         // 
#define GM_ADC_1KHz                        (0<<0)       // ADC samples each 1 ms
#define GM_ADC_500Hz                       (1<<0)       // ADC samples each 2 ms
#define GM_ADC_250Hz                       (2<<0)       // ADC samples each 4 ms

#endif /* _APP_REGS_H_ */