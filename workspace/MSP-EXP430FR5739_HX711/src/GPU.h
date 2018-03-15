/*******************************************************************************
 *
 ******************************************************************************/
#ifndef GPU_INCLUDED
#define GPU_INCLUDED

// Odd parity included
#define STX (0x02)
#define ETX (0x83)
#define ACK (0x86)

#define GPU_BUFF_SIZE 128

volatile unsigned int GPU_RX_OK;

void GPU_Rx( char input );
void GPU_Tx();

int  GPU_Check();
int  GPU_Process();

char GPU_Calculate_BCC(char* pc_Buff, unsigned int ui_Size);
char GPU_Calculate_OddParity( char c_Byte );

#endif // GPU_INCLUDED
