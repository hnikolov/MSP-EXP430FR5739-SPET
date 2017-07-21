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
#define MODE_6             6
#define MODE_7             7
#define MODE_8             8
#define NOT_VALID         15
#define LAST_MODE         MODE_8

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

//#define MCU_CLK_8MHZ // Values from User Guide for 8MHz
#define UART_BR    52
#define UART_BRF    1
#define UART_BRS 0x49
#define UART_OS16   1

// TODO: Define constants for 8MHz SMCLK (8MHz)
// 7.95 / uS (measured)
// 40KHz carrier = 25uS; constant = 200 @ 8MHz
#define MICROSECONDS_25 199
#define KHz_40 199

//===============================================================
// Bit time @ baudrate 1200 for 8MHz SMCLK
// T = 832 us (6656 but tuned to 6624)
#define T_1200_BAUD 6624
#define HALF_T_1200_BAUD (T_1200_BAUD / 2)
// Probably not needed
#define NOMINAL_HALF_BIT_TIME_ONE_1200_BAUD (     T_1200_BAUD / 2) // 416 us
#define NOMINAL_BIT_TIME_ZERO_1200_BAUD     (     T_1200_BAUD    )

#define HALF_BIT_TIME_ONE_1200_BAUD_LO_LIM  (     T_1200_BAUD / 4) //  208 us
#define HALF_BIT_TIME_ONE_1200_BAUD_HI_LIM  (3U * T_1200_BAUD / 4) //  624 us
#define BIT_TIME_ZERO_1200_BAUD_LO_LIM      (3U * T_1200_BAUD / 4) //  624 us
#define BIT_TIME_ZERO_1200_BAUD_HI_LIM      (6U * T_1200_BAUD / 4) // 1248 us

//===============================================================
// Bit time @ baudrate 2400 for 8MHz SMCLK
// T = 416 us (3328 but tuned to 3312)
#define T_2400_BAUD 3312
#define HALF_T_2400_BAUD (T_2400_BAUD / 2)
// Probably not needed
#define NOMINAL_HALF_BIT_TIME_ONE_2400_BAUD (     T_2400_BAUD / 2)
#define NOMINAL_BIT_TIME_ZERO_2400_BAUD     (     T_2400_BAUD    )

#define HALF_BIT_TIME_ONE_2400_BAUD_LO_LIM  (     T_2400_BAUD / 4)
#define HALF_BIT_TIME_ONE_2400_BAUD_HI_LIM  (3U * T_2400_BAUD / 4)
#define BIT_TIME_ZERO_2400_BAUD_LO_LIM      (3U * T_2400_BAUD / 4)
#define BIT_TIME_ZERO_2400_BAUD_HI_LIM      (6U * T_2400_BAUD / 4)

//===============================================================
// Bit time @ baudrate 4800 for 8MHz SMCLK
// T = 208 us (1664 but tuned to 1656)
#define T_4800_BAUD 1656
#define HALF_T_4800_BAUD (T_4800_BAUD / 2)
// Probably not needed
#define NOMINAL_HALF_BIT_TIME_ONE_4800_BAUD (     T_4800_BAUD / 2)
#define NOMINAL_BIT_TIME_ZERO_4800_BAUD     (     T_4800_BAUD    )

#define HALF_BIT_TIME_ONE_4800_BAUD_LO_LIM  (     T_4800_BAUD / 4)
#define HALF_BIT_TIME_ONE_4800_BAUD_HI_LIM  (3U * T_4800_BAUD / 4)
#define BIT_TIME_ZERO_4800_BAUD_LO_LIM      (3U * T_4800_BAUD / 4)
#define BIT_TIME_ZERO_4800_BAUD_HI_LIM      (6U * T_4800_BAUD / 4)

//===============================================================
// Set the proper constants to be used
//===============================================================
#define T_BAUD                       T_1200_BAUD
#define HALF_T_BAUD                 (     T_BAUD / 2)

#define NOMINAL_HALF_BIT_TIME_ONE   (     T_BAUD / 2)
#define NOMINAL_BIT_TIME_ZERO       (     T_BAUD    )

#define HALF_BIT_TIME_ONE_LO_LIM    (     T_BAUD / 4)
#define HALF_BIT_TIME_ONE_HI_LIM    (3U * T_BAUD / 4)
#define BIT_TIME_ZERO_LO_LIM        (3U * T_BAUD / 4)
#define BIT_TIME_ZERO_HI_LIM        (6U * T_BAUD / 4)
//===============================================================

// 8.3 / millisecond (7.84?) @ ALKC = VLO (10KHz), used with OneShotTimer()
#define MILLISECONDS_12   100
#define MILLISECONDS_30   249
#define MILLISECONDS_40   330
#define MILLISECONDS_500  4150
#define SECONDS_1         8300

extern volatile unsigned int  ADCResult;
extern volatile unsigned char active;
extern volatile unsigned char ULP;
extern volatile unsigned char mode;
extern volatile unsigned char UserInput;


extern volatile unsigned char PWM_Flag;

volatile unsigned char ThreshRange[3]; // TODO

// UART
volatile char char_RX;
volatile unsigned int UART_RX_OK;

// Used to send data via an output pin
volatile char byte_TX;
volatile static char BPM_Buffer_TX[8] = {0};
volatile int byte_c; // Byte Counter
volatile int bit_c;  // Bit  Counter
volatile int transmit_length; // Byte counter Mode 8

// TODO: Timer capture variables
volatile unsigned int StartTime, EndTime;
volatile int time;
volatile int start;
volatile unsigned char RollBack2Zero;

// Function Declarations
// TODO: Why extern?
extern void Init_System(void);
extern void Init_UART_9600(void);
extern void Init_UART_19200(void);
extern void Init_UART_115200(void);

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
        while (!(UCA0IFG&UCTXIFG)); // USCI_A0 TX buffer ready?
        UCA0TXBUF = uc_pBuff[i];
    }
}

// Pin PWM (IR) =================================================================
inline void enable_Pin_PWM()
{
    P1OUT  &= ~BIT5;                     // Set P1.5 to 0 (low) when PWM is stopped
//    P1DIR  |=  BIT5;                     // P1.5 output
    P1SEL0 |=  BIT5;                     // P1.5 options select

    TB0CCR0  = KHz_40-1;                 // PWM Period = 25 uS @ SMCLK (8MHz)

    TB0CCTL2 = OUTMOD_3;                 // CCR2 7:reset/set; 3:set/reset
    TB0CCR2  = KHz_40 >> 1;              // CCR2 50% PWM duty cycle
//    TB0CTL   = TBSSEL_2 + MC_1 + TBCLR;  // SMCLK (8MHz), up mode, clear TAR

    // Used to alter modulated output/silence ("envelop")
    TB2CCR0  = HALF_T_BAUD;              // Represents half Bit duration
    TB2CCTL0 = CCIE;
    TB2CTL   = TBSSEL_2 + MC_1 + TBCLR;  // SMCLK (8MHz), up mode, clear TAR
}

inline void disable_Pin_PWM()
{
    P1SEL0  &= ~BIT5;               // Makes P1.5 a regular I/O pin
    P1DIR   &= ~BIT5;               // P1.5 input (Note: To avoid 'parasitic' output)
    TB0CTL   = 0;                   // Stop the timer

    TB2CCTL0 = 0;
    TB2CTL   = 0;
}

inline void init_IR_Rx()
{
    // Configure Port Pin
    P1DIR  &= ~BIT0;                            // P1.0/TA0.1 Input Capture
    P1SEL0 |=  BIT0;                            // TA0.1 option select

//    P1REN  &= ~BIT0;                            // No pull-up/down resistors
    P1REN  |=  BIT0;                            // Pull-up/down resistors enabled
    P1OUT  |=  BIT0;                            // Pull-up resistor
}

inline void enable_IR_Rx()
{
    // Configure the TA0CCR1 to do input capture
    TA0CCTL1 = CAP + CM_3 + CCIE + SCS + CCIS_0;
    // TA0CCR1 Capture mode; Both Rising and Falling Edge; Interrupt enable; Synchronized; CCI1A

    TA0CTL |= TASSEL_2 + MC_2 + TACLR + TAIE; // SMCLK, Cont Mode; clear timer, int. enable on roll over
    // TAIE enables the TAIFG interrupt request
}

inline void disable_IR_Rx()
{
    // Disable input capture
    TA0CCTL1 = 0;
    TA0CTL   = 0;
}

#endif // FR_EXP_INCLUDED
