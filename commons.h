#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#ifdef UART_BASE_DEBUG
char g_debug_vals[500] = {0};
char g_debug_pos = 0;
#endif

#define READY_SSI while(!(HWREG(SSI1_BASE + SSI_O_SR) & SSI_SR_TFE)){}
#define WRITE_SSI(x) HWREG(SSI1_BASE + SSI_O_DR) = (x);

