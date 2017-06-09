/*******************************************************************************
 *
 * BPM-related code to transfer data via a pin (IR)
 *
 ******************************************************************************/
#include "BPM.h"

void Byte_Tx_IR( char ch_Byte )
{
    // TODO: UCA0IE &= ~UCRXIE; // Disable UART RX Interrupt?
    byte_TX = ch_Byte;

    enable_Pin_PWM();

    __bis_SR_register( LPM2_bits );      // Enter LPM2
    __no_operation();                    // For debugger

    disable_Pin_PWM();
    // TODO: UCA0IE |= UCRXIE; // Enable UART RX Interrupt?
}

// Note: Valid ui_Size has to be checked before the call
void IR_TX_Data( volatile char *uc_pBuff, unsigned int ui_Size )
{
    // TODO: Send preamble, start, and stop bits; consider odd parity as well
    byte_c = ui_Size; // Number of bytes to be sent

    enable_Pin_PWM();

//    sendBits(0x03, 3); // Preamble
//    __bis_SR_register( LPM2_bits );      // Enter LPM2
//    __no_operation();

    unsigned int i = 0;
    for( i = 0; i < ui_Size; i++ )
    {
//        sendBits(0x01, 1); // Start
//        __bis_SR_register( LPM2_bits );      // Enter LPM2
//        __no_operation();

        byte_TX = uc_pBuff[i];
        __bis_SR_register( LPM2_bits );      // Enter LPM2
        __no_operation();

//        sendBits(0x00, 1); // Stop
//        __bis_SR_register( LPM2_bits );      // Enter LPM2
//        __no_operation();
    }
    disable_Pin_PWM();
}
