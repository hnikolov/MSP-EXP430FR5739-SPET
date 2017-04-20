/*******************************************************************************
 * GPU envelop:
 * <STX> <PLD> <ETX> <BCC>
 * <PLD> can be several characters
 * Len determined by <ETX> (0x03)
 * BCC = XOR ( <PLD> <ETX> ); 7 bits
 *
 ******************************************************************************/

#include "GPU.h"
#include "FR_EXP.h"


static char GPU_Buffer[BUFF_SIZE] = {0};
static unsigned int ui_Cntr = 0;

typedef enum
{
    st_STX = 0,
    st_PLD,
    st_BCC

} my_state_t;

my_state_t state = st_STX;

void GPU_Rx( char input )
{
    GPU_RX_OK = 0;

    switch(state)
    {
        case st_STX:
            if( input == STX )
            {
                ui_Cntr = 0;
                GPU_Buffer[ui_Cntr] = input;
                ui_Cntr++;
                state = st_PLD;
            }
            break;

        case st_PLD:
            // This should not happen
            if( ui_Cntr > BUFF_SIZE-1 )      { state = st_STX; }

            // Read STX data; stay at PLD
            if( input == STX )               { ui_Cntr = 0;    }

            // Read ETX data; switch to BCC
            else if( input == ETX )          { state = st_BCC; }

            GPU_Buffer[ui_Cntr] = input;
            ui_Cntr++;

            break;

        case st_BCC:
            GPU_Buffer[ui_Cntr] = input;
            ui_Cntr++;
            GPU_RX_OK = 1;
            state = st_STX; // Protocol message received
            break;

        default: // Unknown state; Should not happen
            state = st_STX;
            break;
    }
}

// Send back the received GPU message
void GPU_Tx()
{
    UART_TX_Char('\n');
    UART_TX_Data(GPU_Buffer, ui_Cntr);
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
