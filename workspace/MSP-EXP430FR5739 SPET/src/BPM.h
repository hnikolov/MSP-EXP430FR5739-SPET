/*******************************************************************************
 *
 ******************************************************************************/
#ifndef BPM_INCLUDED
#define BPM_INCLUDED

#include "FR_EXP.h"

void Byte_Tx_IR( char ch_Byte );
void IR_TX_Data( volatile char *uc_pBuff, unsigned int ui_Size );


inline int getBit()
{
    static char idx = 7;
    idx++;
    if( idx == 8 ) { idx = 0; } // LSB first

    return (byte_TX >> idx) & 0x01;
}

#endif // BPM_INCLUDED
