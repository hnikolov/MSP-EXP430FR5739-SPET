/*******************************************************************************
 *
 ******************************************************************************/
#ifndef FR_EXP_INCLUDED
#define FR_EXP_INCLUDED

#include "msp430fr5739.h"

// Mode Definitions
#define NO_MODE            0
#define MODE_1             1
#define MODE_2             2
#define MODE_3             3
#define MODE_4             4
#define MODE_5             5
#define NOT_VALID         15
#define LAST_MODE         MODE_5

// Pin Definitions
#define ACC_PWR_PIN       BIT7
#define ACC_PWR_PORT_DIR  P2DIR
#define ACC_PWR_PORT_OUT  P2OUT
#define ACC_PORT_DIR      P3DIR
#define ACC_PORT_OUT      P3OUT
#define ACC_PORT_SEL0     P3SEL0
#define ACC_PORT_SEL1     P3SEL1
#define ACC_X_PIN         BIT0
#define ACC_Y_PIN         BIT1
#define ACC_Z_PIN         BIT2

// Accelerometer Input Channel Definitions
#define ACC_X_CHANNEL     ADC10INCH_12
#define ACC_Y_CHANNEL     ADC10INCH_13
#define ACC_Z_CHANNEL     ADC10INCH_14

static const unsigned char LEDs[] = {BIT7, BIT6, BIT5, BIT4, BIT3, BIT2, BIT1, BIT0};
#define LED1              0
#define LED2              1
#define LED3              2
#define LED4              3
#define LED5              4
#define LED6              5
#define LED7              6
#define LED8              7

#define UART_BUFF_SIZE 7

// 8.75 / millisecond ?
#define MILLISECONDS_30   260
#define MILLISECONDS_40   350

extern volatile unsigned int  ADCResult;
extern volatile unsigned char active;
extern volatile unsigned char ULP;
extern volatile unsigned char mode;
extern volatile unsigned char UserInput;

volatile unsigned char ThreshRange[3]; // TODO

static char RXChar;

// Function Declarations
// TODO: Why extern?
extern void Init_System(void);
extern void Init_UART(void);

extern void LedSequence( unsigned int ui_Repeats );
extern void RunningLight(void);

extern unsigned int CalibrateADC(void);
extern void TakeADCMeas(void);

extern void SetupAccel(void); // setup ADC for Acc
extern void ShutDownAccel(void);

extern void SetupThermistor(void);
extern void ShutDownTherm(void);

void LED_Flash(unsigned char LEDn, unsigned int nTimes);
//extern void LEDsOff();
//extern void DisableSwitches(void);
//extern void EnableSwitches(void);

extern void StartDebounceTimer(unsigned char);
extern void LongDelay(void);
extern void OneShotTimer( unsigned int ticks );

//extern void TXData(void);
//extern void TXBreak(unsigned char);
//extern void UART_TX_Data(char *uc_pBuff, unsigned int ui_Size);

inline void Init_LEDs()
{
    // P3.4- P3.7 are set as output, low
    P3OUT &= ~(BIT4 + BIT5 + BIT6 + BIT7);
    P3DIR |=   BIT4 + BIT5 + BIT6 + BIT7;

    // PJ.0,1,2,3 are set as output, low
    PJOUT &= ~(BIT0 + BIT1 + BIT2 + BIT3);
    PJDIR |=   BIT0 + BIT1 + BIT2 + BIT3;
}

// Inlined because used in ISR
// P3.4- P3.7 are set low (output)
// PJ.0,1,2,3 are set low (output)
inline void LEDsOff()
{
    P3OUT &= ~(BIT4 + BIT5 + BIT6 + BIT7);
    PJOUT &= ~(BIT0 + BIT1 + BIT2 + BIT3);
}

// #188-D pointless comparison of unsigned integer with zero
// if( (LEDn >= 0) && (LEDn <= 3) ) { P3OUT = LEDs[LEDn]; }
inline void LED_On(unsigned char LEDn)
{
    if(  LEDn <= 3                 ) { P3OUT |= LEDs[LEDn]; }
    if( (LEDn >= 4) && (LEDn <= 7) ) { PJOUT |= LEDs[LEDn]; }
}

inline void LED_Off(unsigned char LEDn)
{
    if(  LEDn <= 3                 ) { P3OUT &= ~LEDs[LEDn]; }
    if( (LEDn >= 4) && (LEDn <= 7) ) { PJOUT &= ~LEDs[LEDn]; }
}

inline void LED_Toggle(unsigned char LEDn)
{
    if(  LEDn <= 3                 ) { P3OUT ^= LEDs[LEDn]; }
    if( (LEDn >= 4) && (LEDn <= 7) ) { PJOUT ^= LEDs[LEDn]; }
}

// Switches ===================================================================
inline void Init_Switches()
{
    // P4.0 and P4.1 are configured as switches
    // Port 4 has only two pins
    P4OUT |=   BIT0 + BIT1;        // Configure pullup resistor
    P4DIR &= ~(BIT0 + BIT1);       // Direction = input
    P4REN |=   BIT0 + BIT1;        // Enable pullup resistor
    P4IES &= ~(BIT0 + BIT1);       // P4.0 Lo/Hi edge interrupt
    P4IE   =   BIT0 + BIT1;        // P4.0 interrupt enabled
    P4IFG  = 0;                    // P4 IFG cleared
}

inline void DisableSwitches(void)
{
    P4IFG = 0;                     // P4 IFG cleared
    P4IE &= ~(BIT0+BIT1);          // P4.0 interrupt disabled
    P4IFG = 0;                     // P4 IFG cleared
}

inline void EnableSwitches(void)
{
    P4IFG = 0;                     // P4 IFG cleared
    P4IE  = BIT0+BIT1;             // P4.0 interrupt enabled
}

// UART =========================================================================
inline void UART_TX_Char( char aChar )
{
    while (!(UCA0IFG&UCTXIFG));    // USCI_A0 TX buffer ready?
    UCA0TXBUF = aChar;
}

// Note: Valid ui_Size has to be checked before the call
inline void UART_TX_Data(char *uc_pBuff, unsigned int ui_Size)
{
    unsigned int i = 0;
    for( i = 0; i < ui_Size; i++ )
    {
        while (!(UCA0IFG&UCTXIFG));           // USCI_A0 TX buffer ready?
        UCA0TXBUF = uc_pBuff[i];
    }
}

#endif // FR_EXP_INCLUDED
