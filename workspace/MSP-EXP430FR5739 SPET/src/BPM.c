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
    enable_Pin_PWM();

    sendBits(0x07, 3);
    // sendPreamble_24() // To be enabled after testing with the Oscilloscope

    byte_c = ui_Size;                   // Number of data bytes to be sent

    unsigned int i = 0;
    for( i = 0; i < ui_Size; i++ )
    {
        sendBits(0x00, 1);              // Start bit

        byte_TX = uc_pBuff[i];          // Data bytes
        __bis_SR_register( LPM2_bits ); // Enter LPM2
        __no_operation();               // For debugger

        sendBits(0x01, 1);              // Stop bit
    }
    sendBits(0x01, 1);                  // Frame Stop Bit

    disable_Pin_PWM();
}
