/*******************************************************************************
 *
 * FR_EXP.c
 * User Experience Code for the MSP-EXP430FR5739
 * C Functions File
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
 ******************************************************************************/
#include "msp430fr5739.h"
#include "FR_EXP.h"

//volatile unsigned char ThreshRange[3] = {0};

/**********************************************************************//**
 * @brief  Initialises system
 * 
 * @param  none 
 *  
 * @return none
 *************************************************************************/
void Init_System(void)
{
    // Startup clock system in max. DCO setting ~8MHz
    // This value is closer to 10MHz on untrimmed parts
    CSCTL0_H = 0xA5;                          // Unlock register
    CSCTL1  |= DCOFSEL0 + DCOFSEL1;           // Set max. DCO setting
    CSCTL2   = SELA_1   + SELS_3 + SELM_3;    // set ACLK = vlo; MCLK = DCO
    CSCTL3   = DIVA_0   + DIVS_0 + DIVM_0;    // set all dividers
    CSCTL0_H = 0x01;                          // Lock Register
  
    // Turn off temp.
    REFCTL0 |=  REFTCOFF;
    REFCTL0 &= ~REFON;
  
    // Enable switches
    Init_Switches();
    
    // Enable LEDs
    Init_LEDs();
    
    // UART 0
    Init_UART_9600();
//    Init_UART_19200();

    // P2.7 is used to power the voltage divider for the NTC thermistor
    P2OUT &= ~BIT7;
    P2DIR |=  BIT7;

    // P1.4 is used as input from NTC voltage divider
    // Set it to output low
    P1OUT &= ~BIT4;
    P1DIR |=  BIT4;

    // ----------------------
    // Terminate Unused GPIOs
    // ----------------------
    // P1.0 - P1.7 is unused
    P1OUT &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT5 + BIT6 + BIT7);
    P1DIR &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT5 + BIT6 + BIT7);
    P1REN |=  (BIT0 + BIT1 + BIT2 + BIT3 + BIT5 + BIT6 + BIT7);
  
    // P2.2 - P2.6 is unused
    P2OUT &= ~(BIT2 + BIT3 + BIT4 + BIT5); // + BIT6); // TODO pin 6 as pwm
    P2DIR &= ~(BIT2 + BIT3 + BIT4 + BIT5); // + BIT6);
    P2REN |=  (BIT2 + BIT3 + BIT4 + BIT5); // + BIT6);

    // P3.0, P3.1 and P3.2 are accelerometer inputs
    P3OUT &= ~(BIT0 + BIT1 + BIT2);
    P3DIR &= ~(BIT0 + BIT1 + BIT2);
    P3REN |=   BIT0 + BIT1 + BIT2;
    
    // PJ.0,1,2,3 are used as LEDs
    // crystal pins for XT1 are unused
    PJOUT &= ~(BIT4 + BIT5);
    PJDIR &= ~(BIT4 + BIT5);
    PJREN |=   BIT4 + BIT5;
}

void LedSequence( unsigned int ui_Repeats )
{
    unsigned char counter = ui_Repeats;

    LEDsOff();
    while( counter > 0 )
    {
        RunningLight();
        counter--;
    }
    LEDsOff();
}

void RunningLight(void)
{
    unsigned int  flag = 4;

    unsigned char LED_ArrayPJ[] = {0x01,0x02,0x04,0x08};
    unsigned char LED_ArrayP3[] = {0x80,0x40,0x20,0x10};

    while( flag > 0 ) // From the outside - in
    {
        P3OUT = LED_ArrayP3[flag-1];
        PJOUT = LED_ArrayPJ[flag-1];
        OneShotTimer( MILLISECONDS_30 );
        flag--;
    }

    while( flag < 4 )
    {
        P3OUT = LED_ArrayP3[flag];
        PJOUT = LED_ArrayPJ[flag];
        OneShotTimer( MILLISECONDS_30 );
        flag++;
    }
}

void LED_Flash(unsigned char LEDn, unsigned int nTimes)
{
    while( nTimes > 0 )
    {
        LED_Toggle( LEDn );
        OneShotTimer( MILLISECONDS_40 );
        LED_Toggle( LEDn );
        OneShotTimer( MILLISECONDS_40 );
        nTimes--;
    }
}

/**********************************************************************//**
 * @brief  Calibrate thermistor or accelerometer
 * 
 * @param  none 
 *  
 * @return none
 *************************************************************************/
unsigned int CalibrateADC(void)
{
    unsigned char CalibCounter =0;
    unsigned int Value = 0;

    // Disable interrupts & user input during calibration
    DisableSwitches();

    while(CalibCounter < 50)
    {
        P3OUT ^= BIT4;
        CalibCounter++;
        while (ADC10CTL1 & BUSY);
        ADC10CTL0 |= ADC10ENC | ADC10SC ;       // Start conversion
        __bis_SR_register(CPUOFF + GIE);        // LPM0, ADC10_ISR will force exit
        __no_operation();
        Value += ADCResult;
    }
    Value = Value/50;

    // Reenable switches after calibration
    EnableSwitches();
    return Value;
}

/**********************************************************************//**
 * @brief  Take ADC Measurement
 * 
 * @param  none 
 *  
 * @return none
 *************************************************************************/
void TakeADCMeas(void)
{  
    while( ADC10CTL1 & BUSY );
    ADC10CTL0 |= ADC10ENC | ADC10SC;       // Start conversion
    __bis_SR_register(CPUOFF + GIE);       // LPM0, ADC10_ISR will force exit
    __no_operation();                      // For debug only
}

/**********************************************************************//**
 * @brief  Initialises Accelerometer
 * 
 * @param  none 
 *  
 * @return none
 *************************************************************************/
void SetupAccel(void)
{  
    // Setup  accelerometer
    // ~20KHz sampling
    // Configure GPIO
    ACC_PORT_SEL0    |=  ACC_X_PIN  + ACC_Y_PIN + ACC_Z_PIN;    // Enable A/D channel inputs
    ACC_PORT_SEL1    |=  ACC_X_PIN  + ACC_Y_PIN + ACC_Z_PIN;
    ACC_PORT_DIR     &= ~(ACC_X_PIN + ACC_Y_PIN + ACC_Z_PIN);
    ACC_PWR_PORT_DIR |=  ACC_PWR_PIN;                           // Enable ACC_POWER
    ACC_PWR_PORT_OUT |=  ACC_PWR_PIN;

    // Allow the accelerometer to settle before sampling any data
    __delay_cycles(200000);

    // Single channel, once,
    ADC10CTL0 &= ~ADC10ENC;                        // Ensure ENC is clear
    ADC10CTL0  =  ADC10ON     + ADC10SHT_5;
    ADC10CTL1  =  ADC10SHS_0  + ADC10SHP + ADC10CONSEQ_0 + ADC10SSEL_0;
    ADC10CTL2  =  ADC10RES;
    ADC10MCTL0 =  ADC10SREF_0 + ADC10INCH_12;
    ADC10IV    =  0x00;                            // Clear all ADC12 channel int flags
    ADC10IE   |=  ADC10IE0;

    // Setup Thresholds for relative difference in accelerometer measurements
    ThreshRange[0] = 25;
    ThreshRange[1] = 50;
    ThreshRange[2] = 75;
}

/**********************************************************************//**
 * @brief  ShutDownAccel
 * 
 * @param  none 
 *  
 * @return none
 *************************************************************************/
void ShutDownAccel(void)
{  
      ACC_PORT_SEL0    &= ~(ACC_X_PIN + ACC_Y_PIN + ACC_Z_PIN);
      ACC_PORT_SEL1    &= ~(ACC_X_PIN + ACC_Y_PIN + ACC_Z_PIN);
      ACC_PORT_DIR     &= ~(ACC_X_PIN + ACC_Y_PIN + ACC_Z_PIN);
      ACC_PWR_PORT_DIR &= ~ACC_PWR_PIN;
      ACC_PWR_PORT_OUT &= ~ACC_PWR_PIN;
      ADC10CTL0        &= ~(ADC10ENC + ADC10ON);
      ADC10IE          &= ~ADC10IE0;
      ADC10IFG          = 0;
}

/**********************************************************************//**
 * @brief  Setup thermistor
 * 
 * @param  none 
 *  
 * @return none
 *************************************************************************/
void SetupThermistor(void)
{
  // ~16KHz sampling
  //Turn on Power
  P2DIR  |= BIT7;
  P2OUT  |= BIT7;
  
  // Configure ADC
  P1SEL1 |= BIT4;  
  P1SEL0 |= BIT4; 
  
  // Allow for settling delay 
  __delay_cycles(50000);
  
  // Configure ADC
  ADC10CTL0 &= ~ADC10ENC; 
  ADC10CTL0  =  ADC10SHT_7 + ADC10ON;        // ADC10ON, S&H=192 ADC clks
  // ADCCLK  =  MODOSC = 5MHz
  ADC10CTL1  =  ADC10SHS_0 + ADC10SHP + ADC10SSEL_0;
  ADC10CTL2  =  ADC10RES;                    // 10-bit conversion results
  ADC10MCTL0 =  ADC10INCH_4;                // A4 ADC input select; Vref=AVCC
  ADC10IE    =  ADC10IE0;                      // Enable ADC conv complete interrupt
  
  // Setup Thresholds for relative difference in Thermistor measurements
  ThreshRange[0] = 15;
  ThreshRange[1] = 25;
  ThreshRange[2] = 45;
}

/**********************************************************************//**
 * @brief  ShutDownTherm
 * 
 * @param  none 
 *  
 * @return none
 *************************************************************************/
void ShutDownTherm(void)
{
    // Turn off Vcc
    P2DIR &= ~BIT7;
    P2OUT &= ~BIT7;

    // Turn off ADC Channel
    P1SEL1 &= ~BIT4;
    P1SEL0 &= ~BIT4;

    // Turn off ADC
    ADC10CTL0 &= ~(ADC10ENC + ADC10ON);
    ADC10IE   &= ~ADC10IE0;
    ADC10IFG   =  0;
}

/**********************************************************************//**
 * @brief  Sets up the Timer A1 as debounce timer 
 * 
 * @param  delay: pass 0/1 to decide between 250 and 750 ms debounce time 
 *  
 * @return none
 *************************************************************************/
void StartDebounceTimer(unsigned char delay)
{  
    // default delay = 0
    // Debounce time = 1500* 1/8000 = ~200ms
    TA1CCTL0 = CCIE;                          // TACCR0 interrupt enabled
    if( delay ) { TA1CCR0 =  750; }
    else        { TA1CCR0 = 1500; }
    TA1CTL = TASSEL_1 + MC_1;                 // ACLK, up mode
}

/**********************************************************************//**
 * @brief  Long Delay
 * 
 * @param  none 
 *  
 * @return none
 *************************************************************************/
void LongDelay()
{
    __delay_cycles(250000);
    __no_operation();
}

void OneShotTimer( unsigned int ticks )
{
    TA0CCTL0 = CCIE;                    // TACCR0 interrupt enabled
    TA0CCR0  = ticks;                   // 350 ticks = ~40ms?
    TA0CTL   = TASSEL_1 + MC_1;         // ACLK, up mode

    // VLO stays on in LPM4
    __bis_SR_register(LPM4_bits + GIE); // Enter LPM4 w/interrupt
    __no_operation();                   // For debugger

    // Quit after wake up
    TA0CTL   = 0;
    TA0CCTL0 = 0;
}

// ACLK = 32768Hz, SMCLK = 8MHz
/**************************************************************************
 * N = 8_000_000/BaudRate
 * UCA0BR0 = INT( N )
 * if N > 16
 *     UCOS16 = 1
 *     UCBRx  = INT(N/16)
 *     UCBRFx = INT([(N/16) – INT(N/16)] × 16)
 *     UCBRSx = Look-up Table (18-4), Fractional part of N
 */
void Init_UART_9600()
{
    // Configure UART pins P2.0 & P2.1
    P2SEL1 |=   BIT0 + BIT1;
    P2SEL0 &= ~(BIT0 + BIT1);

    // Configure UART 0
    UCA0CTL1 = UCSWRST;                       // Place UCA0 in Reset to be configured

    UCA0CTL1  |= UCSSEL_2;                      // Set SMCLK as UCLk
//    UCA0BR0   = 52;                            // 9600 baud: N = 833.3333; INT(N/16) = 52
//    UCA0BR1   = 0;
    UCA0BRW   = 52;

    // UCOS16 = 1
    // UCBRFx = (52.0833 - 52) * 16 = 1.33 = 1
    // UCBRSx = 0x49,  (Refer User Guide, Table 18-4 for fraction .3333)
    //UCA0MCTLW = UCBRF_1 + (0x49 * UCBRS0) + UCOS16;
    UCA0MCTLW = 0x4911 ;



    UCA0CTL1 &= ~UCSWRST;                     // release from reset
}

void Init_UART_4800()
{
    // Configure UART pins P2.0 & P2.1
    P2SEL1 |=   BIT0 + BIT1;
    P2SEL0 &= ~(BIT0 + BIT1);

    // Configure UART 0
    UCA0CTL1  = UCSWRST;                       // Place UCA0 in Reset to be configured
    UCA0CTL1 |= UCSSEL_2;                      // Set SMCLK as UCLk
    UCA0BRW   = 116;
    UCA0MCTLW = 0xD6A1 ;
    UCA0CTL1 &= ~UCSWRST;                     // release from reset
}

void Init_UART_19200()
{
    // Configure UART pins P2.0 & P2.1
    P2SEL1 |=   BIT0 + BIT1;
    P2SEL0 &= ~(BIT0 + BIT1);

    // Configure UART 0
    UCA0CTL1  = UCSWRST;                       // Place UCA0 in Reset to be configured
    UCA0CTL1 |= UCSSEL_2;                      // Set SMCLK as UCLk

    UCA0BRW   = 26;                            // 19200 baud: N = 416.6667; INT(N/16) = 26

    // UCOS16 = 1
    // UCBRFx = (26.0417 - 26) * 16 = 6.672 = 6
    // UCBRSx = 0xD6,  (Refer User Guide, Table 18-4 for fraction .6667)
    UCA0MCTLW = 0xD661 ;


//    UCA0BRW = 26;
//    UCA0MCTLW = UCBRF_1 + (0xD6 * UCBRS0) + UCOS16;

    UCA0CTL1 &= ~UCSWRST;                     // release from reset
}

void Init_UART_115200()
{
    // Configure UART pins P2.0 & P2.1
    P2SEL1 |=   BIT0 + BIT1;
    P2SEL0 &= ~(BIT0 + BIT1);

    // Configure UART 0
    UCA0CTL1 |= UCSWRST;
    UCA0CTL1  = UCSSEL_2;                     // Set SMCLK as UCLk
    UCA0BR0   = 4 ;                           // 11520 baud: N = 69.444; INT(N/16) = 4.3402
    UCA0BR1   = 0;
    // UCOS16 = 1
    // UCBRFx = (69.4444 - 69) * 16 = 7.1104 = 7
    // UCBRSx = 0x55,  (Refer User Guide, Table 18-4 for fraction N-INT(N): .4444)
    UCA0MCTLW = 0x5571 ;

    UCA0CTL1 &= ~UCSWRST;                     // release from reset
}
