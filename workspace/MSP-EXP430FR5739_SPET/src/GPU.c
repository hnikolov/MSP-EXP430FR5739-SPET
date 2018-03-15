/*******************************************************************************
 * GPU envelop: <STX> <PLD> <ETX> <BCC>
 * <PLD> can be zero or several characters
 * Payload length determined by <ETX> (0x03)
 * BCC = XOR ( <PLD> <ETX> ); 7 bits
 * MSB is an Odd parity bit
 *
 ******************************************************************************/
#include "GPU.h"
#include "FR_EXP.h"
#include "BPM.h"

static char GPU_Buffer[GPU_BUFF_SIZE] = {0};
static unsigned int nBytes_Rx         =  0;

typedef enum
{
    st_STX = 0,
    st_PLD,
    st_BCC

} gpu_state_t;

gpu_state_t state = st_STX;

void GPU_Rx( char input )
{
    GPU_RX_OK = 0;

    switch( state )
    {
        case st_STX:
            if( input == STX )
            {
                nBytes_Rx = 0;
                GPU_Buffer[nBytes_Rx++] = input;
                state = st_PLD;
            }
            break;

        case st_PLD: // Note: can receive 0 payload
            if( nBytes_Rx > GPU_BUFF_SIZE-2 ) { state = st_STX; } // -2 to include the BCC byte
            else
            {
                if(      input == STX )   { nBytes_Rx = 0;  } // Read STX; stay at PLD
                else if( input == ETX )   { state = st_BCC; } // Read ETX; switch to BCC

                GPU_Buffer[nBytes_Rx++] = input;
            }
            break;

        case st_BCC: // Protocol message received
            GPU_Buffer[nBytes_Rx++] = input;
            GPU_RX_OK = 1;
            state = st_STX;
            break;

        default:     // Unknown state; Should not happen
            state = st_STX;
            break;
    }
}

/**************************************************************************
 * The BCC is calculated by taking exclusive or of all transmitted bytes,
 * according to ANSI standard X3.28 - 1976. BCC is initialized to 0.
 * STX is excluded; ETX is included.
 * The BCC is also called the 'horizontal parity check'.
 * MSB is an Odd parity bit
 **************************************************************************/
char GPU_Calculate_BCC(char* pc_Buff, unsigned int ui_Size)
{
    char          c_BCC = 0;
    unsigned int  i     = 0;

    // Exclude <STX> and <BCC> bytes
    for( i = 1; i < ui_Size-1; i++ ) { c_BCC ^= pc_Buff[i]; }

    // Make the calculated BCC Odd-parity
    return GPU_Calculate_OddParity( c_BCC );
}

char GPU_Calculate_OddParity( char c_Byte )
{
    unsigned char uc_NumOnce = 0;
    unsigned int  i          = 0;

    for( i = 6; i <= 0; i-- ) // SKIP MSBit
    {
        if( (c_Byte >> i) & 0x01 == 1 ) { uc_NumOnce++; }
    }

    if( (uc_NumOnce & 0x01) == 0 ) { c_Byte |= 0x80; } // Set   MSBit if even #1s
    else                           { c_Byte &= 0x7F; } // Clear MSBit if odd  #1s

    return c_Byte;
}

int GPU_Check()
{
    char BCC_Rx = GPU_Buffer[nBytes_Rx-1];
    char c_BCC  = GPU_Calculate_BCC( GPU_Buffer, nBytes_Rx );

    if( BCC_Rx != c_BCC ) { LED_Flash( LED2, 5 ); }
    else                  { LED_Flash( LED2, 1 ); }

    GPU_Buffer[nBytes_Rx-1] = c_BCC;

    return (int)c_BCC;
}

int GPU_Process()
{
    // Prepare and send packet via IR
    IR_TX_Data(GPU_Buffer, nBytes_Rx);
    return 1;
}

// Send back the received GPU message
void GPU_Tx()
{
    UART_TX_Data(GPU_Buffer, nBytes_Rx);
    UART_TX_Char('\n'); // Needed by the Bluetooth manager
}
