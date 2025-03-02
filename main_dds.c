//*****************************************************************************
//
// DDS General-Purpose Output Controller
//
// Copyright (c) 2016 by Colin Parker
//
//
// PINS E1 = reset
// B5 = I/O Update
// B2 = sync out (not used anymore)
// D1 = FSS (held low)
// D0,D2,D3 = SSI
// C4 = UART
// A2,3,4 B6 = PWM
//*****************************************************************************

#include "data_io_dds.h"

inline uint32_t* get_val_addr(uint32_t mask_type, int k_chan) {
    switch (mask_type) {
        case RAMP_AMPLITUDE: return &(g_ampl_cache[k_chan]);
        case RAMP_FREQUENCY: return &(g_freq_cache[k_chan]);
        case RAMP_RDW      : return &(g_dith_cache[k_chan]);
        default            : return &g_default_ptr_target;
    }
}

inline void read_absl_val(int i_slot, uint32_t *p_absl_val) {
    switch ( g_table_poly_dur[i_slot][g_chkpt_pos[i_slot]] ) { // jumps to a specific values
        case -4 ... -1:
            *p_absl_val = g_table_poly_val[i_slot][g_chkpt_pos[i_slot]];
            advance_chkpt_pos(i_slot);
            break;
        default: // increment values
            *p_absl_val += g_table_poly_val[i_slot][g_chkpt_pos[i_slot]];
            advance_frame_pos(i_slot);
            break;
    }
    recede_chkpt_pos(i_slot);
}

// Update the values of other signal channels from the mask the of the current command.
inline void update_absl_val(int k_chan, uint32_t *p_absl_val, uint32_t mask_chan, uint32_t mask_type) {
    uint32_t *p_absl_val_k;
    k_chan++;
    for (; k_chan < N_CHAN; k_chan++)
        if ((0x10 << k_chan) & mask_chan) {
            p_absl_val_k = get_val_addr(mask_type, k_chan);
            *p_absl_val_k = *p_absl_val;
        }
}
/*---------------Handlers-----------------*/
// It's actually writing the final command, but the final command is always made to jump back to the initial value
void write_init_vals(void) {
    int n_chkpt_lookback, chkpt_pos_i, k_chan, i_slot;
    uint32_t mask_i, mask_chan_i, mask_type_i, mask_chan_k, *p_absl_val;
    p_absl_val = &g_default_ptr_target;

    for ( n_chkpt_lookback = 6; n_chkpt_lookback > 0; n_chkpt_lookback-- ) {
        for ( i_slot = 0; i_slot < N_SLOT; i_slot++ ) {
            chkpt_pos_i = g_n_chkpt[i_slot] - n_chkpt_lookback;
            mask_i = g_table_mask[i_slot][chkpt_pos_i];
            mask_type_i = mask_i & MASK_TYPE;
            mask_chan_i = mask_i & MASK_CHAN;
            // If it is a jump (which is true for the last update command)
            if ( chkpt_pos_i < 0 || g_table_poly_dur[i_slot][chkpt_pos_i] > -4 || !mask_i )
                continue;
            for ( k_chan = 0; k_chan < N_CHAN; k_chan++ ) {
                mask_chan_k = 0x10 << k_chan;
                p_absl_val = get_val_addr(mask_type_i, k_chan);
                // For each channel, if it is included in the mask
                if ( mask_chan_k & mask_chan_i ) {
                    // Locate the address of the cache subjugated to update
                    p_absl_val = get_val_addr(mask_type_i, k_chan);
                    // Read the absolute value, increment shouldn't be possible, this is the line different from below
                    *p_absl_val = g_table_poly_val[i_slot][chkpt_pos_i];
                    // For the other chans also i-masked
                    update_absl_val(k_chan, p_absl_val, mask_chan_i, mask_type_i);
                    // Write through the SSI for all i-masked channels as well
                    write_absl_val_ssi(mask_chan_i, p_absl_val, mask_type_i);
                    // At most one k_chan is involved in writing for every i_slot
                    // that's the first included in the mask_chan_k
                    break;
                }
            }
        }
    }
}

void handleUART(void) {
    bool keep_going = true;
    uint32_t char_prefix = 0;
    uint32_t mask = 0;
    uint32_t pos = 0, i_slot = 0;
    g_checksum_uart = 0;
    UARTIntClear(RACK_UART_BASE, UARTIntStatus(RACK_UART_BASE, true));
    char_prefix = readu8_uart();
    keep_going = (char_prefix == 0xAA);
    while ( keep_going ) {
        char_prefix = readu8_uart();

        if ( char_prefix == 'U' ) {
            // Update one channel's arrays to have new values
            i_slot = readu8_uart();
            mask = readu8_uart();

            pos = read16_uart();
            pos = (pos >= MAX_POS) ? MAX_POS - 1 : pos;

            // Receive and update increment durations and values
            g_table_mask[i_slot][pos] = mask;
            g_table_poly_dur[i_slot][pos] = read32_uart();
            g_table_poly_val[i_slot][pos] = read32_uart();
        }
        else if ( char_prefix == 'R' ) { // Set the array index to a specific value for a channel
            i_slot = readu8_uart();
            g_chkpt_pos[i_slot] = read16_uart();
        }
        else if ( char_prefix == 'Z' ) { // Zero out a channel's command list
            i_slot = readu8_uart();
            for ( pos = 0; pos < MAX_POS; pos++ ) {
                g_table_poly_dur[i_slot][pos] = 0;
                g_table_poly_val[i_slot][pos] = 0;
            }
        }
        else if ( char_prefix == 'E' ) { // Change the effective array length
            i_slot = readu8_uart();
            g_n_chkpt[i_slot] = read16_uart();
        }
        else if ( char_prefix == 'T' ) {
            GPIOPinTypeUART(RACK_UART_PORT, RACK_UART_TX_PIN);
            SysCtlDelay(RACK_INIT_WAIT);
            write_checksum_uart();
            UARTIntClear(RACK_UART_BASE, UARTIntStatus(RACK_UART_BASE, true));
            while(UARTBusy(RACK_UART_BASE));
            GPIOPinTypeGPIOInput(RACK_UART_PORT, RACK_UART_TX_PIN);
            keep_going = false;
        }
    }
    write_init_vals();
}

void GPIOIntHandler(void)
{
    uint32_t mask_i, mask_chan_i, mask_type_i, mask_chan_k, *p_absl_val;
    p_absl_val = &g_default_ptr_target;
    int i_slot, active_now, k_chan;
    static enum { PHASE_IS_GOOD, PHASE_IS_HOLDING_ZERO, PHASE_IS_NOT_RESET } phase_status = PHASE_IS_NOT_RESET;

    GPIOIntClear(RACK_CLK_BASE,0x1FF); // clear all interrupts

    if ( !GPIOPinRead(RACK_CLK_BASE, RACK_CLK_PIN) ) {
        return;
    }

    GPIOPinWrite(UPDATE_BASE, UPDATE_PIN, UPDATE_PIN);
    SysCtlDelay(RACK_SIGNAL_WAIT);
    GPIOPinWrite(UPDATE_BASE, UPDATE_PIN, 0);

    active_now = GPIOPinRead(START_BASE, START_PIN);
    if ( active_now == 0 ) {
        g_t_start = FRAMES_TO_SKIP;
        switch (phase_status) {
            case PHASE_IS_NOT_RESET :
                set_phase_ssi(true);
                phase_status = PHASE_IS_HOLDING_ZERO;
                break;
            case PHASE_IS_HOLDING_ZERO :
                set_phase_ssi(false);
                phase_status = PHASE_IS_GOOD;
                break;
        }
    } else if ( g_t_start > 0 ) {
        g_t_start--;
        phase_status = PHASE_IS_NOT_RESET;
    }

    for ( i_slot = 0; i_slot < N_SLOT; i_slot++ ) {
        if ( !g_t_start ) {
            mask_i = g_table_mask[i_slot][g_chkpt_pos[i_slot]];
            mask_type_i = mask_i & MASK_TYPE;
            mask_chan_i = mask_i & MASK_CHAN;
            // If nothing is required in this frame
            if ( !mask_i ) {
                read_absl_val(i_slot, p_absl_val); // just advance, read to nothing
                continue;
            }
            for ( k_chan = 0; k_chan < N_CHAN; k_chan++ ) {
                mask_chan_k = 0x10 << k_chan;
                // For each channel, if it is included in the mask
                if ( mask_chan_k & mask_chan_i ) {   
                    // Locate the address of the cache subjugated to update                 
                    p_absl_val = get_val_addr(mask_type_i, k_chan);
                    // Read the absolute value from increment parameter table into p_absl_val
                    read_absl_val(i_slot, p_absl_val);
                    // For the other chans also i-masked
                    update_absl_val(k_chan, p_absl_val, mask_chan_i, mask_type_i);
                    // Write through the SSI for all i-masked channels as well
                    write_absl_val_ssi(mask_chan_i, p_absl_val, mask_type_i);
                    // At most one k_chan is involved in writing for every i_slot
                    // that's the first included in the mask_chan_k
                    break;
                }
            }
        } else {
            clear_pos(i_slot);
        }
    }

    //ADCSequenceEnable(ADC0_BASE,0);
    // check UART status and call UART processing function if needed
    if(UARTCharsAvail(RACK_UART_BASE)) handleUART();
}

int
main(void)
{
    uint32_t myclock;
    //
    // Set the system clock to run at 80Mhz off PLL with external crystal as
    // reference.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
            SYSCTL_OSC_MAIN);
    myclock = SysCtlClockGet();

    SysCtlDelay(3);
    IntMasterDisable();
    //
    // Enable the GPIO ports we will use (B,D,E)
    //

    // PINS E1 = reset
    // B5 = I/O Update
    // B2 = sync out (not used anymore)
    // D1 = FSS (held low)
    // D0,D2,D3 = SSI
    // C4 = UART RX, C5 = UART TX
    // F2,3; B6,7 = PWM pins

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    // Enable SSI1
    GPIOPinConfigure(GPIO_PD0_SSI1CLK);
    GPIOPinConfigure(GPIO_PD1_SSI1FSS);
    GPIOPinConfigure(GPIO_PD2_SSI1RX);
    GPIOPinConfigure(GPIO_PD3_SSI1TX);
    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3);

    // Set GPIO C4 as UART pin
    GPIOPinConfigure(GPIO_PC4_U1RX);
    GPIOPinTypeUART(RACK_UART_PORT, RACK_UART_RX_PIN);
    GPIOPinConfigure(GPIO_PC5_U1TX);
    GPIOPinTypeGPIOInput(RACK_UART_PORT, RACK_UART_TX_PIN);

    // Enable PWM, use pin PB6, PB7, PF2, PF3
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    GPIOPinTypePWM(PP0_BASE, PP0_PIN | PP1_PIN);
    GPIOPinTypePWM(PP2_BASE, PP2_PIN | PP3_PIN);

    //
    // Enable the GPIO pins used
    //
    GPIOPinTypeGPIOOutput(UPDATE_BASE, UPDATE_PIN);
    GPIOPinTypeGPIOInput(RACK_CLK_BASE, RACK_CLK_PIN | START_PIN);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, RESET_PIN | GPIO_PIN_2 | GPIO_PIN_3);

    // Set up interrupts
    GPIOIntEnable(RACK_CLK_BASE, RACK_CLK_PIN);
    GPIOIntTypeSet(RACK_CLK_BASE, RACK_CLK_PIN, GPIO_RISING_EDGE);
    IntEnable(INT_GPIOA);

    //
    // Configure UART
    //
    UART9BitAddrSet(RACK_UART_BASE, UART_ADDRESS, 0xFF);
    UART9BitEnable(RACK_UART_BASE);
    UARTConfigSetExpClk(RACK_UART_BASE, myclock, 115200,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                    UART_CONFIG_PAR_ZERO));

    //
    // Empty the UART1 FIFO
    //
    while(UARTCharsAvail(RACK_UART_BASE)){
        UARTCharGetNonBlocking(RACK_UART_BASE);
    }
    //UARTFIFOLevelSet(RACK_UART_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);

    //
    // Enable the UART interrupt.
    //
    //UARTIntEnable(RACK_UART_BASE, UART_INT_9BIT);
    //UARTIntClear(RACK_UART_BASE, UARTIntStatus(RACK_UART_BASE, true));

    SSIIntClear(SSI1_BASE, SSI_TXEOT);
    SSIConfigSetExpClk(SSI1_BASE, myclock, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 20000000, 8);
    SSIEnable(SSI1_BASE);

    // Configure the PWM
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 80);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 78);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 78);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);

    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 80);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 78);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 78);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    PWMOutputState(PWM0_BASE,PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);

    //
    // Write the default state to all the pins (all low) and wait
    //
    GPIOPinWrite(UPDATE_BASE, UPDATE_PIN, 0);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0);
    GPIOPinWrite(GPIO_PORTE_BASE, RESET_PIN | GPIO_PIN_2 |GPIO_PIN_3, 0);
    SysCtlDelay(RACK_INIT_WAIT);

    //
    // Send a master reset and wait
    //
    GPIOPinWrite(RESET_BASE, RESET_PIN, RESET_PIN);
    SysCtlDelay(RACK_INIT_WAIT);
    GPIOPinWrite(RESET_BASE, RESET_PIN, 0);
    SysCtlDelay(RACK_INIT_WAIT);

    //
    // Write the preferred frame format (Channel enabling)
    //
    SSIDataPutNonBlocking(SSI1_BASE, 0x00); //Address of CSR
    SSIDataPutNonBlocking(SSI1_BASE, 0xF2); //Enable all channels and use 3-wire single bit

    SSIDataPutNonBlocking(SSI1_BASE, 0x01); //Address of FR1
    SSIDataPutNonBlocking(SSI1_BASE, 0xA8); //PLL Set to 10, VCO gain is high   0111 0100
    SSIDataPutNonBlocking(SSI1_BASE, 0x00); //                                  0000 0000
    SSIDataPutNonBlocking(SSI1_BASE, 0x00); //                                  0000 0000

    SysCtlDelay(RACK_INIT_WAIT);

    GPIOPinWrite(UPDATE_BASE, UPDATE_PIN, UPDATE_PIN);
    SysCtlDelay(RACK_INIT_WAIT);
    GPIOPinWrite(UPDATE_BASE, UPDATE_PIN, 0);
    SysCtlDelay(RACK_INIT_WAIT);

    //
    // Sets the RF frequency
    //
    SSIDataPutNonBlocking(SSI1_BASE, 0x04); //Address of CFTW0
    SSIDataPutNonBlocking(SSI1_BASE, (INIT_FREQ >> 24) & 0xFF); //Value of 80 MHz with 500 MHz clock (0x28F5C28F)
    SSIDataPutNonBlocking(SSI1_BASE, (INIT_FREQ >> 16) & 0xFF);
    SSIDataPutNonBlocking(SSI1_BASE, (INIT_FREQ >> 8) & 0xFF);
    SSIDataPutNonBlocking(SSI1_BASE, (INIT_FREQ >> 0) & 0xFF);

    SysCtlDelay(RACK_INIT_WAIT);

    //
    // Sets the amplitude
    //
    SSIDataPutNonBlocking(SSI1_BASE, 0x06); //Address of ACR
    SSIDataPutNonBlocking(SSI1_BASE, 0x00); //amplitude ramp rate 0
    SSIDataPutNonBlocking(SSI1_BASE, 0x10 | ((INIT_AMP >> 8) & 0x3)); //multiplier enable and 2 MSB = 0x0
    SSIDataPutNonBlocking(SSI1_BASE, INIT_AMP & 0xFF); // total multiplier = 2 = min
    SysCtlDelay(RACK_INIT_WAIT);

    //
    // Sets the PWM ramp end point
    //
    SSIDataPut(SSI1_BASE, 0x0A); // Address of CW1
    SSIDataPut(SSI1_BASE, INIT_RAMP_END >> 24); // Load INIT_RAMP_END into CW1
    SSIDataPut(SSI1_BASE, INIT_RAMP_END >> 16);
    SSIDataPut(SSI1_BASE, INIT_RAMP_END >> 8);
    SSIDataPut(SSI1_BASE, INIT_RAMP_END);
    SysCtlDelay(RACK_INIT_WAIT);

    // Setup frequency sweeps
    SSIDataPut(SSI1_BASE, 0x03); // Address of CFR
    SSIDataPut(SSI1_BASE, CFR_HIGH);
    SSIDataPut(SSI1_BASE, CFR_MID);
    SSIDataPut(SSI1_BASE, CFR_LOW);
    SysCtlDelay(RACK_INIT_WAIT);

    //RDW = 2525000; //0x418937; // 500 KHz
    SSIDataPut(SSI1_BASE, 0x08); // Address of 	RDW
    SSIDataPut(SSI1_BASE, INIT_RDW >> 24); // Load RDW
    SSIDataPut(SSI1_BASE, INIT_RDW >> 16);
    SSIDataPut(SSI1_BASE, INIT_RDW >> 8);
    SSIDataPut(SSI1_BASE, INIT_RDW);
    SysCtlDelay(RACK_INIT_WAIT);

    SSIDataPut(SSI1_BASE, 0x07); // Address of RSRR/FSRR
    SSIDataPut(SSI1_BASE, 0x01); // Load FSRR
    SSIDataPut(SSI1_BASE, 0x01); // Load RSRR
    SysCtlDelay(RACK_INIT_WAIT);

    SSIDataPut(SSI1_BASE, 0x09); // Address of FDW
    SSIDataPut(SSI1_BASE, 0x7FFFFFFF >> 24); // Load FDW
    SSIDataPut(SSI1_BASE, 0x7FFFFFFF >> 16);
    SSIDataPut(SSI1_BASE, 0x7FFFFFFF >> 8);
    SSIDataPut(SSI1_BASE, 0x7FFFFFFF);
    SysCtlDelay(RACK_INIT_WAIT);

    //
    // Send an I/O Update
    //

    GPIOPinWrite(UPDATE_BASE, UPDATE_PIN, UPDATE_PIN);
    SysCtlDelay(RACK_INIT_WAIT);
    GPIOPinWrite(UPDATE_BASE, UPDATE_PIN, 0);
    SysCtlDelay(RACK_INIT_WAIT);

    //
    // Setup the ADC
    //
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_FULL, 1);
    ADCHardwareOversampleConfigure(ADC0_BASE, HW_AVG);
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH3);
    ADCSequenceEnable(ADC0_BASE, 0);

    IntMasterEnable();


    while(true)
    {
        //GPIOIntHandler();
    }

}
