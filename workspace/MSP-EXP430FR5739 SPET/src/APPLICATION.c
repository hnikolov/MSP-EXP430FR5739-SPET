/*******************************************************************************
 *
 * Application Code for the MSP-EXP430FR5739
 * 
 ******************************************************************************/
#include "APPLICATION.h"
#include "FR_EXP.h"
#include <string.h>
#include "GPU.h"

extern volatile unsigned int RX_OK;

/**********************************************************************//**
 * @brief  Mode 1
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
void Mode1(void)
{
    char str_Mode[] = "Mode 1: char echo\n";
    UART_TX_Data(str_Mode, strlen(str_Mode));

    LEDsOff();
    LED_On( LED1 );

    UCA0IE |= UCRXIE; // Enable UART RX Interrupt

    while( mode == MODE_1 )
    {
        // Wait in LPM4 for an interrupt (key pressed)
        __bis_SR_register(LPM4_bits + GIE); // Enter LPM4 w/interrupt
        __no_operation();                   // For debugger
    } // end while() loop

    LED_Off( LED1 );
}


/**********************************************************************//**
 * @brief  Mode 2
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
void Mode2(void)
{
    char str_Mode[] = "Mode 2: GPU echo: <STX> <PLD> <ETX> <BCC>\n";
    UART_TX_Data(str_Mode, strlen(str_Mode));

    LEDsOff();
    LED_On( LED2 );

    UCA0IE |= UCRXIE; // Enable UART RX Interrupt

//    while( UserInput == 0 )
    while( mode == MODE_2 )
    {
        if( GPU_RX_OK == 1 )
        {
            GPU_Check();
            GPU_Process();
            GPU_Tx();
            GPU_RX_OK = 0;
        }
        // Wait in LPM4 after every character received
        __bis_SR_register(LPM4_bits + GIE); // Enter LPM4 w/interrupt
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
    TB0CTL   = TBSSEL_1 + MC_1;         // ACLK, up mode

    //-----------------------
    __bis_SR_register(LPM4_bits + GIE); // Enter LPM4, enable interrupts

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
    TB0CCR2  = 500;                            // CCR2 PWM duty cycle
    TB0CTL   = TBSSEL_2 + MC_1 + TBCLR;         // SMCLK (8MHz), up mode, clear TAR

    // TODO: Use i timer to generate the carrier @ 40KHz
    // Used to alter the duty cycle of TB0CCR2
    TB2CCR0  = 500;
    TB2CCTL0 = CCIE;
    TB2CTL   = TBSSEL_2 + MC_1 + TBCLR;       // SMCLK (8MHz), up mode, clear TAR

    __bis_SR_register(LPM4_bits);             // Enter LPM0
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
    char str_Mode[] = "Mode 5: char echo\n";
    UART_TX_Data(str_Mode, strlen(str_Mode));

    LedSequence( 2 );
    LED_On( LED5 );

    UCA0IE |= UCRXIE; // Enable UART RX Interrupt

    while( mode == MODE_5 )
    {
        // Wait in LPM4 for an interrupt (key pressed)
        __bis_SR_register(LPM4_bits + GIE); // Enter LPM4 w/interrupt
        __no_operation();                   // For debugger
    } // end while() loop

    UCA0IE &= ~UCRXIE; // Disable UART RX Interrupt

    LED_Off( LED5 );
}



/**********************************************************************//**
 * @brief  LED Toggle Sequence for FRAM Writes
 *
 * @param
 * sequence flag
 *
 * @return none
 *************************************************************************/
void LEDSequenceWrite(unsigned int flag)
{
    // The LED sequence fills p based on flag value
    unsigned char LED_ArrayPJ[] = {0x08,0x0C,0x0E,0x0F};
    unsigned char LED_ArrayP3[] = {0x80,0xC0,0xE0,0xF0};

    // LED Sequencing for FRAM writes
    if( flag < 4 )
    {
        PJOUT &= ~(BIT0 +BIT1+BIT2+BIT3);
        P3OUT &= ~(BIT4 +BIT5+BIT6+BIT7);
        P3OUT |= LED_ArrayP3[flag];
    }
    else
    {
        PJOUT &= ~(BIT0 +BIT1+BIT2+BIT3);
        P3OUT &= ~(BIT4 +BIT5+BIT6+BIT7);
        // Keep the entire P3 array on
        P3OUT |= LED_ArrayP3[3];
        PJOUT |= LED_ArrayPJ[flag-4];
    }
}

/**********************************************************************//**
 * @brief  LED Toggle Sequence
 *
 * @param
 * DiffValue Difference between calibrated and current measurement
 * temp Direction of difference (positive or negative)
 * @return none
 *************************************************************************/
void LEDSequence(unsigned int DiffValue, unsigned char temp)
{
    // The same scale is used for cold & hot and tilt up/down
    // only the thresholds are different
    P3OUT |= BIT4;                            // Light up the middle LEDs
    PJOUT |= BIT3;

    if(DiffValue < ThreshRange[0])            // Very close to CAL value
    {
        P3OUT |= BIT4;
        PJOUT |= BIT3;
        PJOUT &= ~(BIT0+BIT1+BIT2);
        P3OUT &= ~(BIT7+BIT6+BIT5);
        counter = 0x34;
    }

    if ((DiffValue >=ThreshRange[0]) && (DiffValue < ThreshRange[1]))
    {
        // Light up one LED
        if(temp == UP)                        // Tilt up, temp up
        {
            PJOUT |= BIT2;
            PJOUT &= ~(BIT1+BIT0);
            P3OUT &= ~(BIT7+BIT6+BIT5);
            counter = 5;
        }
        else                                  // Tilt down, temp down
        {
            PJOUT &= ~(BIT0+BIT1+BIT2);
            P3OUT |= BIT5;
            P3OUT &= ~(BIT6+BIT7);
            counter = 2;
        }
    }
    if ((DiffValue >= ThreshRange[1]) && (DiffValue < ThreshRange[2]))
    {
        // Light up two LEDs
        if(temp == UP)                        // Tilt up 2, temp up 2
        {
            PJOUT |= BIT2 + BIT1;
            PJOUT &= ~(BIT0);
            P3OUT &= ~(BIT7+BIT6+BIT5);
            counter = 6;
        }
        else                                  // Tilt down 2, temp down 2
        {
            PJOUT &= ~(BIT2+BIT1+BIT0);
            P3OUT |= BIT5 + BIT6;
            P3OUT &= ~(BIT7);
            counter = 1;
        }
    }
    if (DiffValue > ThreshRange[2])
    {
        // Light up three LEDs
        if(temp == UP)                        // Tilt up 3, temp up 3
        {
            PJOUT |= BIT2 + BIT1 + BIT0;
            P3OUT &= ~(BIT7+BIT6+BIT5);
            counter = 7;
        }
        else                                  // Tilt down 3, temp down 3
        {
            P3OUT |= BIT5+BIT6+BIT7;
            PJOUT &= ~(BIT2+BIT1+BIT0);
            counter = 0;
        }
    }
}
