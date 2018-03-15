/*******************************************************************************
 *
 ******************************************************************************/
#ifndef BPM_INCLUDED
#define BPM_INCLUDED

#include "FR_EXP.h"

#define MIN_NR_OF_PRE_AMBLES 20
#define NOM_NR_OF_PRE_AMBLES 24

#define BPM_BUFF_SIZE 128
volatile char received_byte;

volatile unsigned int BPM_BYTE_RX_OK;
volatile unsigned int BPM_FRAME_RX_OK;

void BPM_Tx();

void Byte_Tx_IR( char ch_Byte );
void IR_TX_Data( volatile char *uc_pBuff, unsigned int ui_Size );

void BPM_Rx( int input );
void BPM_Rx_NO_HALF_ONE( int input ); // preferred

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
inline void sendPreamble()
{
    sendBits(0xFF, 8);
    sendBits(0x0F, 4);
}

inline int detect_bpm_bit( int aTime )
{
    if(      (time >= HALF_BIT_TIME_ONE_LO_LIM) && (time <= HALF_BIT_TIME_ONE_HI_LIM) ) { return  1; } // half one
    else if( (time >  BIT_TIME_ZERO_LO_LIM)     && (time <= BIT_TIME_ZERO_HI_LIM)     ) { return  0; } // zero
    else                                                                                { return -1; } // out of bounds
}

#endif // BPM_INCLUDED
