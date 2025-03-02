#include "config_dds.h"
#include "config_device.h"

bool  g_to_update = false;
// Global Variable Declaration
uint32_t g_ampl_cache[N_CHAN] = { 0 };     // amplitude value
uint32_t g_freq_cache[N_CHAN] = { 0 };  // frequency value
uint32_t g_dith_cache[N_CHAN] = { 0 };  // RDW value

// Related to the data and position of the command tables
uint32_t g_frame_pos[N_CHAN] = { 0 }; // counting index
uint32_t g_chkpt_pos[N_CHAN] = { 0 }; // array index
int32_t g_t_start = FRAMES_TO_SKIP; // Skip some clocks to synchronize devices

// A list of durations (in units of cycles) for increment addition, potentially negative for the direct jump
int32_t g_table_poly_dur[N_SLOT][MAX_POS] = {{-2}, {-2}, {-2}, {-2}};
// Which channel to increment (and whether to do amplitude or frequency)
uint32_t g_table_mask[N_SLOT][MAX_POS] = {{0}, {0}, {0}, {0}};
// Amplitude or frequency value to increment
int32_t g_table_poly_val[N_SLOT][MAX_POS] = {{0xFFFF}, {0xFFFF}, {0xFFFF}, {0xFFFF}};
uint32_t g_n_chkpt[N_CHAN] = {4, 4, 4, 4}; // effective array length
uint32_t g_default_ptr_target = 0;
uint32_t g_checksum_uart = 0; // UART error detection

inline int32_t read32_uart(){ // make a 32 bit number from 8 bit reads
    int32_t val1, val2, val3, val4;
    val1 = UARTCharGet(RACK_UART_BASE);
    val2 = UARTCharGet(RACK_UART_BASE);
    val3 = UARTCharGet(RACK_UART_BASE);
    val4 = UARTCharGet(RACK_UART_BASE);
    g_checksum_uart += val1 + val2 + val3 + val4;
    return (val1 << 24| val2 << 16 | val3 << 8 | val4);
}

inline int32_t read16_uart(){ // make a 16 bit number from 8 bit reads
    int32_t val1, val2;
    val1 = UARTCharGet(RACK_UART_BASE);
    val2 = UARTCharGet(RACK_UART_BASE);
    g_checksum_uart += val1 + val2;
    return (val1 << 8 | val2);
}

inline uint32_t readu8_uart(){ // make a 16 bit number from 8 bit reads
    uint32_t val1;
    val1 = UARTCharGet(RACK_UART_BASE);
    g_checksum_uart += val1;
    return val1;
}

inline int32_t UARTCharGetAndLog(uint32_t uart_base) {
    int32_t val_char;
    val_char = UARTCharGet(uart_base);
#ifdef UART_BASE_DEBUG
    if (uart_base == UART_BASE_DEBUG) {
        g_debug_vals[g_debug_pos] = val_char;
        g_debug_pos++;
        if (g_debug_pos >= 500)
            g_debug_pos = 0;
    }
#endif
    return val_char;
}

inline int32_t UARTCharGetWithWait(uint32_t uart_base) {
    int a = 100000;
    while ( !UARTCharsAvail(uart_base) && (a > 0 )) {
        a--;
        if (a == 0) return -1;
    }
    return (UARTCharGetAndLog(uart_base));
}

inline void write_checksum_uart() {
    int i_tx;
    for(i_tx = 0; i_tx < 4; i_tx++)
        UARTCharPut(RACK_UART_BASE, g_checksum_uart >> ((3 - i_tx) * 8));
}

inline void recede_chkpt_pos(int i_slot) {
    if ( g_chkpt_pos[i_slot] >= g_n_chkpt[i_slot] )
        g_chkpt_pos[i_slot] = g_n_chkpt[i_slot] - 1;
}

inline void advance_chkpt_pos(int i_slot) {
    g_chkpt_pos[i_slot] += 1;
    g_frame_pos[i_slot]  = 0;
}

inline void clear_pos(int i_slot) {
    g_chkpt_pos[i_slot] = 0;
    g_frame_pos[i_slot] = 0;
}

inline void advance_frame_pos(int i_slot) {
    g_frame_pos[i_slot] += 1;
    // Move to next command in the cycle
    if ( g_frame_pos[i_slot] >=
         g_table_poly_dur[i_slot][g_chkpt_pos[i_slot]] ) {
        g_chkpt_pos[i_slot]  += 1;
        g_frame_pos[i_slot] = 0;
    }
}

inline void set_phase_ssi(bool to_clear_phase) {
    READY_SSI
    WRITE_SSI(0x00); // Address of CSR
    WRITE_SSI(0xF2); // All channels, standard 3-wire mode
    WRITE_SSI(0x03); // Address of CFR
    WRITE_SSI(CFR_HIGH);
    WRITE_SSI(CFR_MID);
    WRITE_SSI(CFR_LOW | (CFR_CLEAR_PHASE & to_clear_phase));
}

inline void write_absl_val_ssi(uint32_t mask_chan, uint32_t *p_absl_val, uint32_t mask_type) {
    uint32_t val_word;
    switch (mask_type) {
        case RAMP_AMPLITUDE :
            READY_SSI
            WRITE_SSI(0x00); // Address of CSR
            WRITE_SSI(mask_chan | 0x02);
            WRITE_SSI(0x06); // Address of ACR
            WRITE_SSI(0x00); // amplitude ramp rate 0 (manual ramping - we write the amplitude each time)
            WRITE_SSI(0x10 | ((*p_absl_val >> 24) & 0x3));
            WRITE_SSI(*p_absl_val >> 16); // [?]
            break;
        case RAMP_FREQUENCY :
            READY_SSI
            WRITE_SSI(0x00); // Address of CSR
            WRITE_SSI(mask_chan | 0x02); // Load the channels
            WRITE_SSI(0x04); // Address of CFTW0
            WRITE_SSI(*p_absl_val >> 24); // load S0 into CFTW0
            WRITE_SSI(*p_absl_val >> 16);
            WRITE_SSI(*p_absl_val >> 8);
            WRITE_SSI(*p_absl_val);
            break;
        case RAMP_RDW :
            READY_SSI
            WRITE_SSI(0x00); // Address of CSR
            WRITE_SSI(mask_chan | 0x02); // Load the channels
            val_word = *p_absl_val >> RDW_SHIFT;
            WRITE_SSI(0x08); // Address of   RDW
            WRITE_SSI(val_word >> 24); // Load RDW
            WRITE_SSI(val_word >> 16);
            WRITE_SSI(val_word >> 8);
            WRITE_SSI(val_word);
            break;
    }
}
