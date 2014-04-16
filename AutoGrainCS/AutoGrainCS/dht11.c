/*
 * dht11.c
 *
 * Created: 4/9/2014 8:10:18 PM
 *  Author: ruizj
 */ 

#define F_CPU 8000000L
#include <avr/io.h>
#include "dht11.h"
#include <util/delay.h>



// Return values:
// DHTLIB_OK
// DHTLIB_ERROR_CHECKSUM
// DHTLIB_ERROR_TIMEOUT
int dht11Read(DHT11 *measurement)
{
	// BUFFER TO RECEIVE
	uint8_t bits[5];
	uint8_t cnt = 7;
	uint8_t idx = 0;

	// EMPTY BUFFER
	for (uint8_t i=0; i< 5; i++) bits[i] = 0;

	// REQUEST SAMPLE
	DDRE |= (1 << PINE7); // set PE7 as output
	PORTE &= ~(1 << PINE7); // set PE7 LOW
	_delay_ms(18);
	PORTE |= (1 << PINE7); // set PE7 HIGH
	_delay_us(40); //delay for 40 us
	DDRE &= ~(1 << PINE7); // set PE7 as input

	//TCCR1B |= (1 << CS10); // set up 16-bit timer

	// ACKNOWLEDGE or TIMEOUT
	unsigned int loopCnt = 10000;
	while(!(PINE & (1 << PINE7))) // while pin == LOW
	if (loopCnt-- == 0) return DHTLIB_ERROR_TIMEOUT;

	loopCnt = 10000;
	while(PINE & (1 << PINE7)) // while pin == HIGH
	if (loopCnt-- == 0) return DHTLIB_ERROR_TIMEOUT;

	// READ OUTPUT - 40 BITS => 5 BYTES or TIMEOUT
	for (uint8_t i=0; i<40; i++)
	{
		loopCnt = 10000;
		while(!(PINE & (1 << PINE7))) // while pin == LOW
		if (loopCnt-- == 0) return DHTLIB_ERROR_TIMEOUT;

		//TCNT1 = 0; // Reset timer value

		loopCnt = 10000;
		while(PINE & (1 << PINE7)) // while pin == HIGH
		if (loopCnt-- == 0) return DHTLIB_ERROR_TIMEOUT;
		_delay_us(40);

		//if (TCNT1 >= 319) bits[idx] |= (1 << cnt);
		if (PINE & (1 << PINE7)) {//bits[idx] |= (1 << cnt);
		//if (bit_is_set(PINE, PINE7)){
			PORTD &= (1<<PIND3);
			 bits[idx] |= (1 << cnt);
		}
		if (cnt == 0)   // next byte?
		{
			cnt = 7;    // restart at MSB
			idx++;      // next byte!
		}
		else cnt--;
	}

	// WRITE TO RIGHT VARS
	// as bits[1] and bits[3] are always zero they are omitted in formulas.
	// change to double to provide a decimal number
	measurement->humidity    = bits[0];
	measurement->temperature = bits[2];
	
	int sum = bits[0] + bits[2];

	if (bits[4] != sum) return DHTLIB_ERROR_CHECKSUM;
	
	return DHTLIB_OK;
}
//
// END OF FILE