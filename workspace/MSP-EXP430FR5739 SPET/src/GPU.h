/*******************************************************************************
 *
 ******************************************************************************/
#ifndef GPU_INCLUDED
#define GPU_INCLUDED


#define STX (0x02)
#define ETX (0x03)
#define ACK (0x06)

#define BUFF_SIZE 128

volatile unsigned int GPU_RX_OK;

void GPU_Rx( char input );
void GPU_Tx();

int  GPU_Check();
int  GPU_Process();

char GPU_Calculate_BCC(char* pc_Buff, unsigned int ui_Size);

#endif // GPU_INCLUDED
