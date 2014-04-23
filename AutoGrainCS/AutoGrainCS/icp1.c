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

#define	ICP_PRESCALE ((0 << CS12) | (1 << CS11) | (0 << CS10))	/* prescale /8 */

volatile unsigned int period1[18]; // holds period of each pulse from UTI
volatile unsigned char count; // keeps track of the number of pulses measured
volatile char icp_go; // when 1, ICIE1 enabled, when 0, interrupt off



/**
 * icp_start_time, icp_period
 *
 * State variables for the Demodulator.
 *
 * start_time and period have the same type as the
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
	 * Capture the ICR 
	 */
	icr = ICR1;							/* capture timestamp	*/

			/*
			 * Beginning of pulse: Compute length of preceding period,
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
	TCCR1A = 0;

	/*
	 * Set the interrupt to activate on rising edge and the prescaler
	 */
	TCCR1B	= (1 << ICES1) | ICP_PRESCALE;

	/*
	 *	Enable both the Input Capture interrupt.
	 */
	TIMSK1	|= (1 << ICIE1);

	return;
}

unsigned int getPeriod(unsigned char index)
{
	return period1[index];
}

char icp_continue(void) {
	return icp_go;
}
