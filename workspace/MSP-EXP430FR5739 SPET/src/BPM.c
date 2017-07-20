/*******************************************************************************
 *
 * BPM-related code to transfer data via a pin (IR)
 *
 ******************************************************************************/
#include "BPM.h"
#include "GPU.h"

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
    // TODO: UCA0IE &= ~UCRXIE; // Disable UART RX Interrupt?
    enable_Pin_PWM();

    //sendBits(0x07, 3); // For debug with Oscilloscope
    sendPreamble();

    byte_c = ui_Size;                   // Number of data bytes to be sent

    unsigned int i = 0;
    for( i = 0; i < ui_Size; i++ )
    {
        sendBits(0x00, 1);              // Start bit

        byte_TX = uc_pBuff[i];          // Data bytes
        __bis_SR_register( LPM2_bits ); // Enter LPM2
        __no_operation();               // For debugger

        sendBits(0x03, 2);              // Stop bit
    }

    disable_Pin_PWM();
    // TODO: UCA0IE |= UCRXIE; // Enable UART RX Interrupt?
}

static char BPM_Buffer_RX[BPM_BUFF_SIZE] = {0};
static unsigned int nBytes_Rx            =  0;

typedef enum
{
    st_WAIT_PRE_AMBLES = 0,
    st_WAIT_FOR_START_BIT,
    st_RECEIVING_DATA_BITS

} bpm_state_t;

bpm_state_t bpm_state = st_WAIT_PRE_AMBLES;

void BPM_Rx( int input ) // Input is either zero, half one, or -1 (not valid)
{
    static int preamble_counter = 0;
    static int bit_counter      = 0;
    static int first_half_one   = 0;

    BPM_BYTE_RX_OK  = 0;
    BPM_FRAME_RX_OK = 0;

    switch( bpm_state )
    {
        case st_WAIT_PRE_AMBLES:
            switch( input )
            {
                case 1: preamble_counter += 1; break; // half one

                case 0:
                    if( preamble_counter > MIN_NR_OF_PRE_AMBLES ) // Start bit after preambles
                    {
                        nBytes_Rx      = 0;
                        received_byte  = 0;
                        bit_counter    = 0;
                        first_half_one = 0;
                        bpm_state      = st_RECEIVING_DATA_BITS;
                    }
                    else { preamble_counter = 0; } // Not enough preambles received, keep the same state
                    break;

                // Kind of carrier drop or undefined timer value, keep the same state
                default: preamble_counter = 0; break;
            }
            break;

        case st_WAIT_FOR_START_BIT:
            switch( input )
            {
                case 0:                      // Zero, this is the start bit for a new byte
                    received_byte    = 0;
                    bit_counter      = 0;
                    first_half_one   = 0;
                    bpm_state        = st_RECEIVING_DATA_BITS;
                    break;

                case  1: LED_Toggle( LED5 ); // Another (Third) stop bit or new pre-ambles (Error)
                default:                     // Timer value out of range -> end of BPM frame
                    BPM_FRAME_RX_OK  = 1;
                    preamble_counter = 0;
                    bpm_state        = st_WAIT_PRE_AMBLES;
                    break;
            }
            break;

        case st_RECEIVING_DATA_BITS:
            switch( input )
            {
                case 1: // half one
                    if( first_half_one == 1 ) // Full one
                    {
                        if(      bit_counter == 8 ) { bit_counter++; }  // Stop bit
                        else if( bit_counter == 9 )                     // Second stop bit
                        {
                            BPM_Buffer_RX[ nBytes_Rx ] = received_byte; // Add byte to buffer
                            nBytes_Rx                 += 1;
                            BPM_BYTE_RX_OK             = 1;
                            bpm_state                  = st_WAIT_FOR_START_BIT;
                        }
                        else // Add bit to byte
                        {
                            received_byte >>= 1;
                            received_byte |= 0x80;
                            bit_counter++;
                        }

                        first_half_one = 0;

                    } else { first_half_one = 1; }
                    break;

                case 0: // zero
                    if( first_half_one == 0 ) // Additional check, add bit to byte
                    {
                        received_byte >>= 1;
                        bit_counter++;
                    }
                    else // first_half_one == 1, framing error, ignore bit
                    {
                        first_half_one = 0;
                        bit_counter++;        // TODO: Do we need also to shift?
                        LED_On( LED8 );
                    }
                    break;

                default: // Timer value out of range, framing error?
                    LED_Toggle( LED7 );

                    preamble_counter = 0;
                    bpm_state        = st_WAIT_PRE_AMBLES;
                    break;
            }
            break;

        default: // Unknown state; Should not happen
            LED_Toggle( LED8 );

            preamble_counter = 0;
            bpm_state        = st_WAIT_PRE_AMBLES;
            break;
    }
}

// For debug only
void BPM_Tx()
{
    UART_TX_Data(BPM_Buffer_RX, nBytes_Rx);
    UART_TX_Char('\n'); // Needed by the Bluetooth manager
}

//=========================================================================================
void BPM_Rx_NO_HALF_ONE( int input ) // input is either zero, full one, or -1 (not valid)
{
    static int preamble_counter = 0;
    static int bit_counter      = 0;

    BPM_BYTE_RX_OK = 0;

    switch( bpm_state )
    {
        case st_WAIT_PRE_AMBLES:
            switch( input )
            {
                case 1: preamble_counter += 2; break; // Since input is 2 half ones

                case 0:
                    if( preamble_counter > MIN_NR_OF_PRE_AMBLES ) // Start bit after preambles detected
                    {
                        received_byte = 0x00;
                        bit_counter   = 0;
                        bpm_state     = st_RECEIVING_DATA_BITS;
                    }
                    else { preamble_counter = 0; } // Not enough preambles received, keep the same state
                    break;

                // Kind of carrier drop or undefined timer value, keep the same state
                default: preamble_counter = 0; break;
            }
            break;

        case st_WAIT_FOR_START_BIT:
            switch( input )
            {
                case 0:        // Zero detected, this is the startbit for a new byte
                    received_byte = 0x00;
                    bit_counter   = 0;
                    bpm_state     = st_RECEIVING_DATA_BITS;
                    break;

                case 1: break; // Another stop bit or new pre-ambles = end-of-frame?

                default:       // Timer value out of range
                    preamble_counter = 0;
                    bpm_state        = st_WAIT_PRE_AMBLES;
                    break;
            }
            break;

        case st_RECEIVING_DATA_BITS:
            switch( input )
            {
                case 1:
                    if( bit_counter == 8 ) // The stop bit detected
                    {
                        bpm_state      = st_WAIT_FOR_START_BIT;
                        BPM_BYTE_RX_OK = 1; // TODO: Add byte to buffer
                    }
                    else // Add bit to byte
                    {
                        received_byte >>= 1;
                        received_byte |= 0x80;
                        bit_counter++;
                    }
                    break;

                case 0: // Add bit to byte
                    received_byte >>= 1;
                    bit_counter++;
                    break;

                default: // Timer value out of range
                    preamble_counter = 0;
                    bpm_state        = st_WAIT_PRE_AMBLES;
                    break;
            }
            break;

        default: // Unknown state; Should not happen
            preamble_counter = 0;
            bpm_state        = st_WAIT_PRE_AMBLES;
            break;
    }
}

// CANNOT BE USED BECAUSE the 854 gauge sends 25 (odd) half ones as preambles :(
// getBPM_Bit( ) -----------------------------------------------------------------------------------
/*
unsigned char first_half_one = 0;
if( (time >= HALF_BIT_TIME_ONE_LO_LIM)  && (time <= HALF_BIT_TIME_ONE_HI_LIM) ) // half one detected
{
    if( first_half_one == 0 )
    {
        first_half_one = 1;
    }
    else
    {
        LED_Toggle( LED1 );
        detected_bit   = 1;
        first_half_one = 0;
    }
}
else if( (time > BIT_TIME_ZERO_LO_LIM) && (time <= BIT_TIME_ZERO_HI_LIM) ) // zero detected
{
    if( first_half_one == 0 )
    {
        LED_Toggle( LED7 );
        detected_bit = 0;
    }
    else // framing error, ignore bit
    {
        LED_Toggle( LED3 );
        detected_bit   = -1;
        first_half_one =  0;
    }
}
//        if( (time <  HALF_BIT_TIME_ONE_LO_LIM) || (time >  BIT_TIME_ZERO_HI_LIM) ) { LED_Toggle( LED3 ); } // out of bounds
else // out of bounds detected, ignore bit
{
    LED_Toggle( LED3 );
    detected_bit   = -1;
    first_half_one =  0;
}
*/
