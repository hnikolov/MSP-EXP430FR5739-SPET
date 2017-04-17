#include "GPU.h"
#include "FR_EXP.h"


static char GPU_Buffer[3] = {0};


typedef enum
{
    STX = 0,
    PLD,
    BCC,
    ETX
} my_state_t;

my_state_t state = STX;

void GPU_Rx( char input )
{
    GPU_RX_OK = 0;

    switch(state)
    {
        case STX:
            if( input == 's' )
            {
                GPU_Buffer[0] = input;
                state = PLD;
            }
            else
            {
                state = STX;
            }
            break;

        case PLD:
            if( input == 's' ) // TODO: Proper const
            {
                GPU_Buffer[0] = input;
                state = PLD;
            }
            else
            {
                GPU_Buffer[1] = input;
                state = ETX;
            }
            break;

        case ETX:
            if( input == 'e' )
            {
                GPU_Buffer[2] = input;
                GPU_RX_OK = 1;
            }
            state = STX; // Protocol message received
            break;

        default:
            state = STX;
            break;
    }
}

// Send back the received GPU message
void GPU_Tx()
{
    UART_TX_Char('\n');
    UART_TX_Data(GPU_Buffer, 3);
    UART_TX_Char('\n');
}

int GPU_Check()
{
    // E.g., check BCC
    return 1;
}

int GPU_Process()
{
    // Prepare and send packet via IR
    return 1;
}
