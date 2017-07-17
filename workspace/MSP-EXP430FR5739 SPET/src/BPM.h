/*******************************************************************************
 *
 ******************************************************************************/
#ifndef BPM_INCLUDED
#define BPM_INCLUDED

#include "FR_EXP.h"

void Byte_Tx_IR( char ch_Byte );
void IR_TX_Data( volatile char *uc_pBuff, unsigned int ui_Size );

volatile char idx;

inline int getBit()
{
    idx++; // Must be properly initialised
    if( idx == 8 ) { idx = 0; } // LSB first

    return (byte_TX >> idx) & 0x01;
}

// Used to send 1 to 8 bits (LSB first)
inline void sendBits( char ch_Value, unsigned int num_Bits )
{
    // Store current byte count (volatile variable)
    int temp_byte_c = byte_c;

    // Initialise global (volatile) variables
    byte_c  = 1;
    bit_c   = num_Bits;
    byte_TX = ch_Value;
    idx     = 7;          // Used by getBit()

    __bis_SR_register( LPM2_bits ); // Enter LPM2
    __no_operation();

    // Restore values
    bit_c  = 8;           // Default value of the bit counter (done at ISR end as well)
    idx    = 7;           // Used by getBit()
    byte_c = temp_byte_c; // Byte count
}

// Note: The preamble is 24 half ones (12 bits)
inline void sendPreamble_12()
{
    sendBits(0xFF, 8);
    sendBits(0x0F, 4);
}

#endif // BPM_INCLUDED
