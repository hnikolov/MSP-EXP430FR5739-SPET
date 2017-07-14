/*******************************************************************************
 *
 * Application Code for the MSP-EXP430FR5739
 * 
 ******************************************************************************/
#include "APPLICATION.h"
#include "FR_EXP.h"
#include "GPU.h"
#include "BPM.h"

#include <string.h>

/**********************************************************************//**
 * @brief  Mode 1
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
// NOTE: This mode does not work without the debugger and LPM4, so use LPM2
void Mode1(void)
{
    char str_Mode[] = "Mode 1: char echo\n";
    UART_TX_Data(str_Mode, strlen(str_Mode));

    UART_RX_OK = 0;

    LEDsOff();
    LED_On( LED1 );

    UCA0IE |= UCRXIE; // Enable UART RX Interrupt

    while( mode == MODE_1 )
    {
        if( UART_RX_OK == 1)
        {
            UART_TX_Char( char_RX );
            UART_RX_OK = 0; // Clear the flag
        }

        // Wait in for an interrupt (UART or key pressed)
        __bis_SR_register(LPM2_bits + GIE); // Enter LPM2 w/interrupt
        __no_operation();                   // For debugger
    } // end while() loop

    UCA0IE &= ~UCRXIE; // Disable UART RX Interrupt

    LED_Off( LED1 );
}


/**********************************************************************//**
 * @brief  Mode 2
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
// NOTE: This mode does not work without the debugger and LPM4, so use LPM2
void Mode2(void)
{
    char str_Mode[] = "Mode 2: GPU echo: <STX> <PLD> <ETX> <BCC>\n";
    UART_TX_Data(str_Mode, strlen(str_Mode));

    GPU_RX_OK = 0;
    idx = 7;

    LEDsOff();
    LED_On( LED2 );

    UCA0IE |= UCRXIE; // Enable UART RX Interrupt

    while( mode == MODE_2 )
    {
        if( UART_RX_OK == 1) { GPU_Rx( char_RX ); }

        if( GPU_RX_OK == 1 )
        {
            GPU_Check();
            GPU_Process();
            GPU_Tx();
            GPU_RX_OK = 0; // Clear the flag
        }
        // Wait in LPM2 after every character received
        __bis_SR_register(LPM2_bits + GIE); // Enter LPM2 w/interrupt
        __no_operation();                   // For debugger
    } // end of while() loop

    UCA0IE &= ~UCRXIE; // Disable UART RX Interrupt

    LED_Off( LED2 );
}

void Mode3(void)
{
    char str_Mode[] = "Mode 3: Timers to modulate signal on pin\n";
    UART_TX_Data(str_Mode, strlen(str_Mode));

    LEDsOff();
    LED_On( LED3 );

    // Settings
    P2OUT &= ~BIT6;
    P2DIR |=  BIT6;   // Set P2.6 to output direction

    // Timer 1: carrier
    TB1CCTL0 = CCIE;                    // TACCR0 interrupt enabled
    TB1CCR0  = MILLISECONDS_30;
    TB1CTL   = TBSSEL_1 + MC_1;         // ACLK, up mode

    // Timer 0: envelop
    TB0CCTL0 = CCIE;                    // TBCCR0 interrupt enabled
    TB0CCR0  = SECONDS_1;               // CCR0: Period
    TB0CTL   = TBSSEL_1 + MC_1;         // ACLK (10KHz), up mode

    __bis_SR_register(LPM4_bits + GIE); // Enter LPM4, enable interrupts
    __no_operation();

    TB1CTL   = 0;
    TB1CCTL0 = 0;

    TB0CTL   = 0;
    TB0CCTL0 = 0;

    LED_Off( LED3 );
}

//*******************************************************************************
//  MSP430FR57x Demo - TimerB, PWM TB0.1-2, Up Mode, DCO SMCLK
//
//  Description: This program generates two PWM outputs on P1.4,P1.5 using
//  TimerB configured for up mode. The value in CCR0, 1000-1, defines the PWM
//  period and the values in CCR1 and CCR2 the PWM duty cycles. Using ~1MHz
//  SMCLK as TACLK, the timer period is ~1ms with a 75% duty cycle on P1.4
//  and 25% on P1.5.
//  ACLK = n/a, SMCLK = MCLK = TACLK = 1MHz // TODO
//
//
//           MSP430FR5739
//         ---------------
//     /|\|               |
//      | |               |
//      --|RST            |
//        |               |
//        |     P1.4/TB0.1|--> CCR1 - 75% PWM
//        |     P1.5/TB0.2|--> CCR2 - 25% PWM
//
//   Priya Thanigai
//   Texas Instruments Inc.
//   August 2010
//******************************************************************************
void Mode4(void)
{
    char str_Mode[] = "Mode 4: PWM, duty cycle changed dynamically\n";
    UART_TX_Data(str_Mode, strlen(str_Mode));

    LEDsOff();
    LED_On( LED4 );

//    P1DIR |= BIT4+BIT5;                       // P1.4 and P1.5 output
//    P1SEL0 |= BIT4+BIT5;                      // P1.4 and P1.5 options select
    P1DIR  |= BIT5;                      // P1.5 output
    P1SEL0 |= BIT5;                      // P1.5 options select

    TB0CCR0 = 1000-1;                         // PWM Period = 125.75uS @ SMCLK (8MHz)
    // P1.4 is used as input from NTC voltage divider
    // TB0CCTL1 = OUTMOD_7;                      // CCR1 reset/set
    // TB0CCR1 = 750;                            // CCR1 PWM duty cycle

    TB0CCTL2 = OUTMOD_7;                      // CCR2 reset/set
    TB0CCR2  = 500;                           // CCR2 PWM duty cycle
    TB0CTL   = TBSSEL_2 + MC_1 + TBCLR;       // SMCLK (8MHz), up mode, clear TAR

    // Used to alter the duty cycle of TB0CCR2
    TB2CCR0  = 500;
    TB2CCTL0 = CCIE;
    TB2CTL   = TBSSEL_2 + MC_1 + TBCLR;       // SMCLK (8MHz), up mode, clear TAR

    __bis_SR_register(LPM4_bits);             // Enter LPM4
    __no_operation();                         // For debugger

    //Exit this mode
    TB0CTL   = 0;

    TB2CCTL0 = 0;
    TB2CTL   = 0;

    LED_Off( LED4 );
}

/**********************************************************************//**
 * @brief  Mode 5
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
void Mode5(void)
{
    char str_Mode[] = "Mode 5: PWM, Modulated output @ 40KHz\n";
    UART_TX_Data(str_Mode, strlen(str_Mode));

    LedSequence( 2 );
    LED_On( LED5 );

    byte_TX = 0xA5;                      // Set a byte to be "send" via a pin

    UCA0IE |= UCRXIE;                    // Enable UART RX Interrupt

    P1DIR  |= BIT5;                      // P1.5 output
    P1SEL0 |= BIT5;                      // P1.5 options select
    P1OUT  &= ~BIT5;                     // Set P1.5 to 0 (low) when PWM is stopped

    TB0CCR0  = KHz_40-1;                 // PWM Period = 25 uS @ SMCLK (8MHz)

    TB0CCTL2 = OUTMOD_3;                 // CCR2 7:reset/set; 3:set/reset
    TB0CCR2  = KHz_40 >> 1;              // CCR2 50% PWM duty cycle
    TB0CTL   = TBSSEL_2 + MC_1 + TBCLR;  // SMCLK (8MHz), up mode, clear TAR

    // Used to alter modulated output/silence ("envelop")
    TB2CCR0  = T_2400_BAUD;              // Represents Bit duration 416 uS @ 8MHz, 2400 baudrate
    TB2CCTL0 = CCIE;
    TB2CTL   = TBSSEL_2 + MC_1 + TBCLR;  // SMCLK (8MHz), up mode, clear TAR

    __bis_SR_register(LPM4_bits);        // Enter LPM4
    __no_operation();                    // For debugger

    // Exit this mode
    UCA0IE &= ~UCRXIE;                   // Disable UART RX Interrupt

    TB0CTL   = 0;

    TB2CCTL0 = 0;
    TB2CTL   = 0;

    LED_Off( LED5 );
}

/**********************************************************************//**
 * @brief  Mode 6
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
// NOTE: Make sure no interrupts occur during the data transmission via the pin
void Mode6(void)
{
    char str_Mode[] = "Mode 6: UART to Pin, modulated @ 40 KHz\n";
    UART_TX_Data(str_Mode, strlen(str_Mode));

    UART_RX_OK = 0;

    idx = 7;

    LEDsOff();
    LED_On( LED6 );

    UCA0IE |= UCRXIE; // Enable UART RX Interrupt

    while( mode == MODE_6 )
    {
        if( UART_RX_OK == 1 )
        {
            UART_RX_OK = 0;          // Clear the flag
/*
            byte_c = 1;
            Byte_Tx_IR( char_RX );
/*/
            BPM_Buffer[0] = char_RX;
            BPM_Buffer[1] = char_RX;
            BPM_Buffer[2] = char_RX;
            IR_TX_Data( BPM_Buffer, 3 );

//            sendBits(0x01, 1);
//            sendBits(0x01, 1);
//            sendBits(0x01, 1);
//*/
            UART_TX_Char( char_RX ); // Echo back the received char
        }
        // Wait in LPM2 after every character received
        __bis_SR_register(LPM2_bits + GIE); // Enter LPM2 w/interrupt
        __no_operation();                   // For debugger
    } // end of while() loop

    UCA0IE &= ~UCRXIE; // Disable UART RX Interrupt

    LED_Off( LED6 );
}


/**********************************************************************//**
 * @brief  Mode 7
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
void Mode7(void)
{
    char str_Mode[] = "Mode 7: Timer capture mode\n";
    UART_TX_Data(str_Mode, strlen(str_Mode));

    UART_RX_OK = 0;

    idx = 7;

    LEDsOff();
    LED_On( LED7 );

    UCA0IE &= ~UCRXIE; // Disable UART RX Interrupt

    //===================================================================

    // Configure Port Pins
    P1DIR  &= ~BIT0;                            // P1.0/TA0.1 Input Capture
    P1SEL0 |=  BIT0;                            // TA0.1 option select

    // Configure TA1.1 to output PWM signal
    // Period = 82/32khz = 2.5ms ~ 400Hz Freq
    // TODO: ~200 us period
    P1DIR  |=  BIT5;                     // P1.5/TB0.2 output
    P1SEL0 |=  BIT5;                     // P1.5 options select
    P1OUT  &= ~BIT5;                     // Set P1.5 to 0 (low) when PWM is stopped

    TB0CCR0  = T_2400_BAUD-1;            // PWM Period = 415us

    TB0CCTL2 = OUTMOD_7;                 // CCR2 7:reset/set; 3:set/reset
    TB0CCR2  = HALF_T_2400_BAUD;         // CCR2: PWM duty cycle 50%
    TB0CTL   = TBSSEL_2 + MC_1 + TBCLR;  // ACLK (), up mode, clear TAR

    //==========================================================================
    // Configure the TA0CCR1 to do input capture
    TA0CCTL1 = CAP + CM_3 + CCIE + SCS + CCIS_0;
                                              // TA0CCR1 Capture mode; CCI1A; Both
                                              // Rising and Falling Edge; interrupt enable
    TA0CTL |= TASSEL_2 + MC_2 + TACLR + TAIE;        // SMCLK, Cont Mode; clear timer
    // TACLR Clear the current timer value as well as the divider and mode
    // ID_3 divide clk by 8
    // TAIE enables the TAIFG interrupt request

    // Variable Initialization
    Count = 0x0;
    First_Time = 0x01;
    //===================================================================

    int Period, ON_Period;
    unsigned char DutyCycle;

    REdge1 = 0;
    REdge2 = 0;
    time   = 0;
    start  = 1;

    while( mode == MODE_7 )
    {
        // Wait in LPM2 after every character received
        __bis_SR_register(LPM0_bits + GIE); // Enter LPM2 w/interrupt
        __no_operation();                   // For debugger

        // On exiting LPM0
        if (TA0CCTL1 & COV)  { LED_On( LED8 ); } // Check for Capture Overflow
        // clear COV if set (in register TAxCCTLn)
        if( time <= 0 )      { LED_Toggle( LED2 );   } // Measured time must be > 0
        if( time <= 0 )      { time = 0xFFFF - time; }

//        if( time > 20026 )   { LED_Toggle( LED1 ); }    // 0.125us * 20026 = 2.5ms
//        if( time < 10000 )   { LED_Toggle( LED7 ); }

        if( time > 1400 )   { LED_Toggle( LED1 ); }    // 0.125us * 1400 = 175us
        if( time >  400 )   { LED_Toggle( LED7 ); }

/*        LED_Toggle( LED7 );

        LED_Off( LED1 );
        LED_Off( LED8 );

        if (TA0CCTL1 & COV)                   // Check for Capture Overflow
        {
            LED_On( LED1 );
//            while(1);                         // Loop Forever
        }

        Period    = REdge2 - REdge1;             // Calculate Period
        ON_Period = FEdge  - REdge1;             // On period

        if( ON_Period < 0 ) { LED_On( LED8 ); }

//        DutyCycle = ((unsigned long)ON_Period*100/Period);
        DutyCycle = 25;

        if( DutyCycle != 25 ) { LED_Toggle( LED7 );     }
        else                  { LED_Toggle( LED7 ); }
*/
    } // end of while() loop


    TB0CTL   = 0;

    TA0CCTL1 = 0;
    TA0CTL   = 0;

    LED_Off( LED7 );
}
