/*******************************************************************************
 *
 * main.c
 * User Experience Code for the MSP-EXP430FR5739
 * 
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Created: Version 1.0 04/13/2011
 *          Version 1.1 05/11/2011
 *          Version 1.2 08/31/2011
 *
 *          Hristo      03/24/2017
 *  
 ******************************************************************************/
#include "FR_EXP.h"
#include "APPLICATION.h"
#include "GPU.h"

const unsigned char LED_Menu[] = {0x80,0xC0,0xE0,0xF0,0xF8};
// These global variables are used in the ISRs and in FR_EXP.c
volatile unsigned char mode           = NO_MODE;
volatile unsigned char UserInput      = 0;
volatile unsigned char active         = 0;
volatile unsigned char SwitchCounter  = 0;
volatile unsigned char Switch1Pressed = 0;
volatile unsigned char Switch2Pressed = 0;

volatile unsigned char PWM_Flag       = 0;

void main(void)
{  
    WDTCTL = WDTPW + WDTHOLD;           // Stop WDT
    Init_System();                      // Init the Board
    LedSequence( 5 );                   // Light up LEDs
  
    while(1)
    {
        // Variable initialisation
        active         = 0;
        Switch2Pressed = 0;

        // Wait in LPM4 for user input
        __bis_SR_register(LPM4_bits + GIE); // Enter LPM4 w/interrupt
        __no_operation();			        // For debugger
    
        // Wake up from LPM because user has entered a mode
        switch( mode )
        {
            case MODE_1:   Mode1();  break;
            case MODE_2:   Mode2();  break;
            case MODE_3:   Mode3();  break;
            case MODE_4:   Mode4();  break;
            case MODE_5:   Mode5();  break;

            default:                    // This is not a valid mode (Switch S2 was pressed w/o mode select)
                while( UserInput == 0 ) // Blink LED1 to indicate invalid entry
                {
                    LED_Toggle( LED1 );
                    OneShotTimer( MILLISECONDS_40 );
                }
                break;
        }
    }
}
 
// Interrupt Service Routines
/**********************************************************************//**
 * @brief  Port 4 ISR for Switch Press Detect
 * 
 * @param  none 
 *  
 * @return none
 *************************************************************************/
#pragma vector=PORT4_VECTOR
__interrupt void Port_4(void)
{
    switch(__even_in_range(P4IV,P4IV_P4IFG1))
    {
        case P4IV_P4IFG0: // S1 pressed
            DisableSwitches();
            Switch2Pressed =  0;
            UserInput      =  1;
            P4IFG         &= ~BIT0;                         // Clear P4.0 IFG
            P3OUT          =  LED_Menu[SwitchCounter];
            PJOUT          =  LED_Menu[SwitchCounter];
            SwitchCounter++;

            if( SwitchCounter > 4 )
            {
                SwitchCounter  = 0;
                Switch1Pressed = 1;
            }
            StartDebounceTimer(0);              // Re-enable switches after de-bounce

            if( active == 1 )
            {
                active = 0;
                mode   = NOT_VALID;
                __bic_SR_register_on_exit(LPM4_bits); // To exit an active mode
                __no_operation();
            }
            break;

        case P4IV_P4IFG1: // S2 pressed
            DisableSwitches();
            StartDebounceTimer(0);              // Re-enable switches after de-bounce
            P4IFG &= ~BIT1;                     // Clear P4.1 IFG

            if( active == 1 ) // This is the second time Switch2 is pressed
            {
                active = 0;
                mode   = NOT_VALID;
                __bic_SR_register_on_exit(LPM4_bits); // To exit an active mode
                __no_operation();
            }
            else //{ Switch2Pressed = 1; } // Enter a mode
            {
                active    = 1;
                UserInput = 0;
                // If the counter value is 0 it indicates either Mode 4 or invalid entry
                if(( SwitchCounter == 0 ) && ( Switch1Pressed == 0 ))        { mode = NOT_VALID; }     // no switch1 press - invalid
                if(  SwitchCounter > LAST_MODE )                             { mode = NOT_VALID; }     // too high! this is invalid entry

                if(( SwitchCounter == 0 ) && ( Switch1Pressed > 0  ))        { mode = LAST_MODE;     }  // because counter rolls over after 4
                if(( SwitchCounter  > 0 ) && ( SwitchCounter <= LAST_MODE )) { mode = SwitchCounter; }  // store mode  (this entry is correct)

                // Reset variables
                Switch1Pressed = 0;
                SwitchCounter  = 0;
                __bic_SR_register_on_exit(LPM4_bits); // Exit LPM4
                __no_operation();
            }
            break;

        default:
            break;
    }
}

/**********************************************************************//**
 * @brief  Timer A0 ISR for one-shot timer, 350 == 40ms timer
 *
 * @param  none 
 *  
 * @return none
 *************************************************************************/
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{  
    __bic_SR_register_on_exit( LPM4_bits );
    __no_operation();
}


/**********************************************************************//**
 * @brief Timer A1 ISR for debounce Timer
 * 
 * @param  none 
 *  
 * @return none
 *************************************************************************/
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR(void)
{
   TA1CCTL0 = 0;
   TA1CTL   = 0;
   EnableSwitches();
}

// Timer TB0x0 interrupt service routine (enable timer 1)
#pragma vector = TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void) {
//    __bic_SR_register_on_exit( LPM4_bits );
    PWM_Flag ^= 0x01;
    LED_Off( LED3 );
    P2OUT &= ~BIT6;
}

// Timer TB1x0 interrupt service routine
#pragma vector = TIMER1_B0_VECTOR
__interrupt void TIMER1_B0_ISR(void) {
    if( PWM_Flag == 1 )
    {
//    __bic_SR_register_on_exit( LPM4_bits );
        LED_Toggle( LED3 );
        P2OUT ^= BIT6;
    }
}

// Timer TB2x0 interrupt service routine
int direction = 2;
int flag      = 0;
int prev_flag = 0; // To detect data transition

#pragma vector = TIMER2_B0_VECTOR
__interrupt void TIMER2_B0_ISR(void) {
    if( mode == MODE_4 )
    {
        if( TB0CCR2 > 997 || TB0CCR2 < 3 ) { direction = -direction; }
        TB0CCR2 += direction;
    }
    else if( mode == MODE_5 )
    {
        flag = getBit();
        if( flag == 1 && prev_flag == 0 )      // Bit changes from 0 to 1
        {
            P1SEL0 |= BIT5;                    // Timer output selected
            TB0CTL  = TBSSEL_2 + MC_1 + TBCLR; // SMCLK (8MHz), up mode, clear TAR
        }
        else if( flag == 0 && prev_flag == 1 ) // Bit changes from 1 to 0
        {
            P1SEL0 &= ~BIT5;                   // Makes P1.5 a regular I/O pin (output set to 0)
            TB0CTL  = 0;                       // Stop the timer
        }
        prev_flag = flag;
    }
}

/**********************************************************************//**
 * @brief Echo back RXed character, confirm TX buffer is ready first
 *
 * @param  none
 *
 * @return none
 *************************************************************************/

// General recommendation:
//     Read the value into a global variable and leave the low power mode on exit.
//     Do all the other stuff in the main application.

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCA0IV,0x08))
    {
        case 0:  break;                    // Vector 0 - no interrupt
        case 2:                            // Vector 2 - RXIFG
            RXChar = UCA0RXBUF;

            switch( mode )
            {
                case MODE_1:   UART_RX_OK = 1;
                               __bic_SR_register_on_exit(LPM2_bits); // Exit LPM4
                               __no_operation();                     // For debugger
                               break;
                case MODE_2:   UART_RX_OK = 1;
                               __bic_SR_register_on_exit(LPM2_bits); // Exit LPM4
                               __no_operation();                     // For debugger
                               break;
                case MODE_3:   break;
                case MODE_4:   break;
                case MODE_5:   UART_TX_Char( RXChar );  break;        // echo, do not exit LPM
                default:       break;
            }
            break;
        case 4:  break;                    // Vector 4 - TXIFG
        default: break;
    }
}
