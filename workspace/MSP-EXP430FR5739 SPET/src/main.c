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
    UCA0IE &= ~UCRXIE; // Disable UART RX Interrupt

    switch(__even_in_range(P4IV,P4IV_P4IFG1))
    {
        case P4IV_P4IFG0:
            DisableSwitches();
            Switch2Pressed =  0;
            UserInput      =  1;
            P4IFG         &= ~BIT0;                         // Clear P4.0 IFG
            P3OUT          =  LED_Menu[SwitchCounter];
            PJOUT          =  LED_Menu[SwitchCounter];
            SwitchCounter++;

            if( SwitchCounter > 4 )
            {
                SwitchCounter = 0;
                Switch1Pressed++;
            }
            StartDebounceTimer(0);              // Re-enable switches after de-bounce
            break;
          
        case P4IV_P4IFG1:
            DisableSwitches();
            StartDebounceTimer(0);              // Re-enable switches after de-bounce
            P4IFG &= ~BIT1;                     // Clear P4.1 IFG

            // This is the second time Switch2 is pressed
            // Was the code executing inside of the modes?
            if ((Switch2Pressed > 0) && (active == 1)) { break; }
            else                                       { Switch2Pressed = 1; } // Enter a mode

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

/**********************************************************************//**
 * @brief Echo back RXed character, confirm TX buffer is ready first
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
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
//            while (!(UCA0IFG&UCTXIFG));    // USCI_A0 TX buffer ready?
//            UCA0TXBUF = UCA0RXBUF;         // TX -> RXed character
            RXChar = UCA0RXBUF;

            switch( mode )
            {
                case MODE_1:   UART_TX_Char( RXChar );  break; // echo
                case MODE_2:   GPU_Rx( RXChar );
                               __bic_SR_register_on_exit(LPM4_bits); // Exit LPM4
                               break;
                case MODE_3:   break;
                case MODE_4:   break;
                case MODE_5:   UART_TX_Char( RXChar );  break; // echo
                default:       break;
            }
            break;
        case 4:  break;                    // Vector 4 - TXIFG
        default: break;
    }
}
