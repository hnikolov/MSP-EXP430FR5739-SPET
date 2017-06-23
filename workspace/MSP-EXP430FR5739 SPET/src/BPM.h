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
//    static char idx = 7;
    idx++;
    if( idx == 8 ) { idx = 0; } // LSB first

    return (byte_TX >> idx) & 0x01;
}

// Used to send 1 to 7 bits (LSB first)
inline void sendBits( char ch_Value, unsigned int num_Bits )
{
    // Store current byte count
    int temp_byte_c = byte_c;

    byte_c  = 1;
    bit_c   = num_Bits;
    byte_TX = ch_Value;

    idx = 7; // used by getBit()

//    enable_Pin_PWM(); // TODO: Probably we can use it

    __bis_SR_register( LPM2_bits );      // Enter LPM2
    __no_operation();

//    disable_Pin_PWM(); // TODO

    bit_c  = 8; // Restore the default value of the bit counter (done at ISR end as well)
    idx    = 7; // used by getBit()
    byte_c = temp_byte_c; // Restore byte count
}

#endif // BPM_INCLUDED
