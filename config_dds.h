#include "commons.h"
#include "inc/hw_adc.h"
#include "driverlib/rom.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/watchdog.h"

#define RACK_INIT_WAIT 0x1000
#define RACK_SIGNAL_WAIT 0x4

#define INIT_FREQ (0x28F5C28F) // 80 MHz start
#define INIT_AMP 0x0
#define INIT_RAMP_END 0xFFFFFFFF //0x33333333
#define INIT_RDW 0
#define RDW_SHIFT 7
//#define INIT_RDW 0x14F8B6


#define N_SLOT 4
#define N_CHAN 4
#define MAX_POS 500

#define RAMP_AMPLITUDE 0x01
#define RAMP_FREQUENCY 0x02
#define RAMP_RDW       0x04 //might need to be 0x04

#define MASK_CHAN 0xF0
#define MASK_TYPE 0x0F
#define HW_AVG 2
#define FRAMES_TO_SKIP 3

#define UPDATE_BASE GPIO_PORTB_BASE
#define UPDATE_PIN GPIO_PIN_5

#define RESET_BASE GPIO_PORTE_BASE
#define RESET_PIN GPIO_PIN_1

#define RACK_CLK_BASE GPIO_PORTA_BASE
#define RACK_CLK_PIN GPIO_PIN_3

#define START_BASE GPIO_PORTA_BASE
#define START_PIN GPIO_PIN_2

#define RACK_UART_PORT GPIO_PORTC_BASE
#define RACK_UART_RX_PIN GPIO_PIN_4
#define RACK_UART_TX_PIN GPIO_PIN_5
#define RACK_UART_BASE UART1_BASE

#define PP0_BASE GPIO_PORTB_BASE
#define PP1_BASE GPIO_PORTB_BASE
#define PP3_BASE GPIO_PORTF_BASE
#define PP2_BASE GPIO_PORTF_BASE

#define PP0_PIN GPIO_PIN_6 //PB6
#define PP1_PIN GPIO_PIN_7 //PB7
#define PP2_PIN GPIO_PIN_2 //PF2
#define PP3_PIN GPIO_PIN_3 //PF3

#define CFR_HIGH (0b10 << 6) // Enable frequency sweeps (AFP = 10)
#define CFR_MID  0x43        // Enable linear sweeps, no-dwell bit disabled
#define CFR_LOW  0x00

#define CFR_CLEAR_PHASE 0x02
