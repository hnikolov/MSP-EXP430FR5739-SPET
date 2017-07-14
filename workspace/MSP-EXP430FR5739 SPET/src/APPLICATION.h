/*******************************************************************************
 *
 * Application Code for the MSP-EXP430FR5739
 * 
 ******************************************************************************/
#ifndef APPLICATION_INCLUDED
#define APPLICATION_INCLUDED

#define DOWN 0
#define UP   1

// The FRAM section from 0xD000 - 0xF000 is used by all modes
// for performing writes to FRAM
// Do not use this section for code or data placement.
// It will get overwritten!
#define ADC_START_ADD     0xD400
#define ADC_END_ADD       0xF000
#define FRAM_TEST_START   0xD400
#define FRAM_TEST_END     0xF000
#define MEM_UNIT          0x0200

unsigned char counter;

extern void Mode1(void);
extern void Mode2(void);
extern void Mode3(void);
extern void Mode4(void);
extern void Mode5(void);
extern void Mode6(void);
extern void Mode7(void);

extern void FRAM_Write      (unsigned int StartAddress);
extern void LEDSequence     (unsigned int DiffValue, unsigned char temp);
extern void LEDSequenceWrite(unsigned int flag);

#endif // APPLICATION_INCLUDED
