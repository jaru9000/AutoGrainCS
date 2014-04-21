// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* - File              : icp.c
* - Compiler          : IAR EWAAVR 3.20
*
* - Support mail      : avr@atmel.com
*
* - Supported devices : All devices with an Input Capture Unit can be used.
*                       The example is written for ATmega64
*
* - AppNote           : AVR135 - Pulse-Width Demodulation
*
* - Description       : Routines for use Pulse-Width Demodulation
*
* Originally authored by Bruce McKenny
* $Revision: 1.5 $
* $Date: Wednesday, November 02, 2005 13:20:02 UTC $
*****************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "icp1.h"

/*
 * Definitions for the ICP pin; for this example we use timer 1
 */
#define	ICP_PIN		PIND			/* ICP1 GPIO value	*/
#define	ICP_PORT	PORTD			/* ICP1 GPIO port	*/
#define	ICP_DDR		DDRD			/* ICP1 GPIO DDR	*/
#define	ICP_BIT		PD4				/* ICP1 GPIO pin	*/

volatile unsigned int period1[18];
volatile unsigned char count;
volatile char icp_go;

/*
 * Definitions for ICP timer (1) setup.
 */
#define	ICP_OCR		OCR1A			/* ICP1 Output Compare register		*/
#define	ICP_OC_IE	OCIE1A			/* ICP1 timer Output Compare enable */
#define	ICP_OC_IF	OCF1A			/* ICP1 timer Output Compare flag	*/
#define	ICP_IE		ICIE1			/* ICP1 interrupt enable			*/
#define	ICP_IF		ICF1			/* ICP1 interrupt flag				*/
#define	ICP_CTL_A	TCCR1A			/* ICP1 timer control				*/
#define	ICP_CTL		TCCR1B			/* ICP1 interrupt control			*/
#define	ICP_SENSE	ICES1			/* ICP1 interrupt sense (rising/falling) */
#define	ICP_PRESCALE ((0 << CS12) | (1 << CS11) | (0 << CS10))	/* prescale /8 */

#define	ICP_START_SENSE	(1 << ICP_SENSE)	/* start with rising edge	*/

/**
 * icp_start_time, icp_stop_time, icp_period
 *
 * State variables for the Demodulator.
 *
 * start_time, stop_time and period all have the same type as the
 * respective TCNT register.
 */
unsigned int icp_start_time;
unsigned int icp_period;


/**
 * TIMER1_CAPT()
 *
 * ICP capture interrupt.
 */
ISR(TIMER1_CAPT_vect)
{
	PORTB |= (1 << PINB7); // UTI Power Down High for Active
	unsigned int icr;

	/*
	 * Capture the ICR and then reverse the sense of the capture.
	 * These must be done in this order, since as soon as the
	 * sense is reversed it is possible for ICR to be updated again.
	 */
	icr = ICR1;							/* capture timestamp	*/

			/*
			 * Beginning of pulse: Compute length of preceding period,
			 * and thence the duty cycle of the preceding pulse.
			 */
			icp_period = icr;	/* Length of previous period */
			
			if (count < 18) 
			{
				period1[count++] = icp_period;
			}
			else {
				icp_go = 0;
			}
				
			TCNT1 =  0;

			/*
			 * Update the timeout based on the new period. (The new period
			 * is probably the same as the old, give or take clock drift.)
			 * We add 1 to make fairly sure that, in case of competition,
			 * the PWM edge takes precedence over the timeout.
			 */
			//ICP_OCR = icr + icp_period + 1;		/* Move timeout window		*/
			//TIFR1 = (1 << ICP_OC_IF);			/* Clear in case of race	*/

			/*
			 * Check for a race condition where a (very) short pulse
			 * ended before we could reverse the sense above.
			 * If the ICP pin is still high (as expected) OR the IF is
			 * set (the falling edge has happened, but we caught it),
			 * then we won the race, so we're done for now.
			 */
			//if ((ICP_PIN & (1 << ICP_BIT)) || (TIFR1 & (1 << ICP_IF)))
				//break;

	return;
}



/**
 * icp_init()
 *
 * Set up the ICP timer.
 */
void
icp_init(void)
{
	DDRD |= (1 << PIND4);
	PORTD &= ~(1 << PIND4);
	icp_go = 1;
	count = 0;
	/*
	 * Nothing interesting to set in TCCR1A
	 */
	ICP_CTL_A = 0;

	/*
	 * Set the interrupt sense and the prescaler
	 */
	ICP_CTL	= ICP_START_SENSE | ICP_PRESCALE;

	/*
	 *	Enable both the Input Capture and the Output Capture interrupts.
	 */
	TIMSK1	|= (1 << ICP_IE);

	return;
}

unsigned int getPeriod(unsigned char index)
{
	return period1[index];
}

char icp_continue(void) {
	return icp_go;
}
