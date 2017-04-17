/*******************************************************************************
 *
 ******************************************************************************/
#ifndef GPU_INCLUDED
#define GPU_INCLUDED

volatile unsigned int GPU_RX_OK;

void GPU_Rx( char input );
void GPU_Tx();

int  GPU_Check();
int  GPU_Process();

#endif // GPU_INCLUDED
