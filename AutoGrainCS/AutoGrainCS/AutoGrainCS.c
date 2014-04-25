/*
 * AutoGrainMoistureControl.c
 *
 * Created: 4/9/2014 8:09:45 PM
 *  Author: ruizj
 *
 * Purpose: This program is the main program of the Automated Moisture Control System designed by
 * Jason Ruiz & Thomas Zirkle for the senior design project at Andrews University 2014. It handles 
 * communication via I2C and USB which interfaces with a LabVIEW program. Measurements are taken from
 * sensors connected to the Atmega256rfr2. The clock rate used is 1 MHz. 
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h> // for _no_op
#include <avr/sleep.h>

#include "TWI_Master.h"
#include "dht11.h"
#include "icp1.h"

// Addr. and FT-X commands
#define TWI_GEN_CALL	0x00
#define TWI_targetSlaveAddress	0x22 

// Transmission States
#define SEND_DATA             0x01
#define REQUEST_DATA          0x02
#define READ_DATA_FROM_BUFFER 0x03
#define SUCCESSFUL_TRANSMIT	  0x04

#define messageBuf_size		  0x29
#define TWI_init_messageBuf_size 0x02

// Other definitions
unsigned char size_of_buffer_out;
unsigned char USB_data; // data received from LabVIEW
unsigned char messageBuf[messageBuf_size]; // buffer of data to be transmitted
DHT11 dht; // Struct with DHT11 data
int status; // status of DHT11

unsigned int UTI_read_data[18]; // Data received from UTI

//Int to 8-bit
#define low(x)   ((x) & 0xFF)
#define high(x)   (((x)>>8) & 0xFF)

void Send_Data_to_LabVIEW();
void Get_UTI_Data();

// Redo transmission if NACK is received
// From Atmel
unsigned char TWI_Act_On_Failure_In_Last_Transmission ( unsigned char TWIerrorMsg )
{
	// A failure has occurred, use TWIerrorMsg to determine the nature of the failure
	// and take appropriate actions.
	// Se header file for a list of possible failures messages.
	
	// Here is a simple sample, where if received a NACK on the slave address,
	// then a retransmission will be initiated.
	
	if ( (TWIerrorMsg == TWI_MTX_ADR_NACK) | (TWIerrorMsg == TWI_MRX_ADR_NACK) )
	TWI_Start_Transceiver();
	
	return TWIerrorMsg;
}

/* Main portion of program. Checks for request from LabVIEW and writes
   sensor measurements back using the TWI method. */ 
int main(void) 
{
	DDRB |= (1 << PINB7); //UTI_PD ... OUTPUT
	TWI_Master_Initialise();
	sleep_disable();
	
	//enable interrupts
	sei();

    while(1)
    {
      // Check if Send_Data command is in from LabVIEW
	  messageBuf[0] = (TWI_targetSlaveAddress<<TWI_ADR_BITS) | (TRUE<<TWI_READ_BIT);
      TWI_Start_Transceiver_With_Data(messageBuf,TWI_init_messageBuf_size);
	  TWI_Get_Data_From_Transceiver( messageBuf, TWI_init_messageBuf_size ); // Read I2C's data
      
     
	      // Check if the TWI Transceiver has completed an operation.
		      // Check if the last operation was successful
		      if ( TWI_statusReg.lastTransOK )
		      {
			      
				      USB_data = messageBuf[1];        // Store data. 

				   if (USB_data == 0xFF)
				   {
					 status = dht11Read(&dht);
					 sei();
					 Get_UTI_Data();
					 Send_Data_to_LabVIEW();
				   }
				   
			  }
		      
		      else // Got an error during the last transmission
		      {
			      // Use TWI status information to determine cause of failure and take appropriate actions.
			      TWI_Act_On_Failure_In_Last_Transmission( TWI_Get_State_Info( ) );
		      }
	      
	      
	      
	      // Do something else while waiting for the TWI Transceiver to complete the current operation
	      //__no_operation(); // Put own code here.
		  _NOP();
      }
	  
   // }
}

// Function to send data back to LabVIEW
void Send_Data_to_LabVIEW()
{ 
	
	//messageBuf[0] = TWI_targetSlaveAddress;		// Use Gen. Call to activate I2C chip and tell it MCU will be writing
	TWI_Start_Transceiver_With_Data(messageBuf,TWI_init_messageBuf_size);
	
					// Prepare data to send
					messageBuf[0] = (TWI_targetSlaveAddress<<TWI_ADR_BITS) | (FALSE<<TWI_READ_BIT);
					messageBuf[1] = high(dht.humidity);
					messageBuf[2] = low(dht.humidity);
					messageBuf[3] = high(dht.temperature); 
					messageBuf[4] = low(dht.temperature);  
					messageBuf[5] = high(UTI_read_data[0]);
					messageBuf[6] = low(UTI_read_data[0]);
					messageBuf[7] = high(UTI_read_data[1]);
					messageBuf[8] = low(UTI_read_data[1]);
					messageBuf[9] = high(UTI_read_data[2]);
					messageBuf[10] = low(UTI_read_data[2]);
					messageBuf[11] = high(UTI_read_data[3]);
					messageBuf[12] = low(UTI_read_data[3]);
					messageBuf[13] = high(UTI_read_data[4]);
					messageBuf[14] = low(UTI_read_data[4]);
					messageBuf[15] = high(UTI_read_data[5]);
					messageBuf[16] = low(UTI_read_data[5]);
					
					messageBuf[17] = high(UTI_read_data[6]);
					messageBuf[18] = low(UTI_read_data[6]);
					messageBuf[19] = high(UTI_read_data[7]);
					messageBuf[20] = low(UTI_read_data[7]);
					messageBuf[21] = high(UTI_read_data[8]);
					messageBuf[22] = low(UTI_read_data[8]);
					messageBuf[23] = high(UTI_read_data[9]);
					messageBuf[24] = low(UTI_read_data[9]);
					messageBuf[25] = high(UTI_read_data[10]);
					messageBuf[26] = low(UTI_read_data[10]);
					messageBuf[27] = high(UTI_read_data[11]);
					messageBuf[28] = low(UTI_read_data[11]);
					
					messageBuf[29] = high(UTI_read_data[12]);
					messageBuf[30] = low(UTI_read_data[12]);
					messageBuf[31] = high(UTI_read_data[13]);
					messageBuf[32] = low(UTI_read_data[13]);
					messageBuf[33] = high(UTI_read_data[14]);
					messageBuf[34] = low(UTI_read_data[14]);
					messageBuf[35] = high(UTI_read_data[15]);
					messageBuf[36] = low(UTI_read_data[15]);
					messageBuf[37] = high(UTI_read_data[16]);
					messageBuf[38] = low(UTI_read_data[16]);
					messageBuf[39] = high(UTI_read_data[17]);
					messageBuf[40] = low(UTI_read_data[17]);
					
					TWI_Start_Transceiver_With_Data( messageBuf, messageBuf_size ); // Send data to slave
					while ( TWI_Transceiver_Busy() ); // Should wait for completion.
				 

			if (!TWI_statusReg.lastTransOK )  // Got an error during the last transmission
			{
				// Use TWI status information to determine cause of failure and take appropriate actions.
				TWI_Act_On_Failure_In_Last_Transmission( TWI_Get_State_Info( ) );
			}

				USB_data = 0x00;
		
	
}


// Function that gets data from UTI
void Get_UTI_Data()
{	
	PORTB |= (1 << PINB7); // UTI Power Down High for Active
	icp_init(); // begin UTI reading
 
	int go = 1;
	while (go == 1) { go = icp_continue();} // continue getting data until specified amount is obtained
	TIMSK1	&= (~(1 << ICIE1)); //disable interrupt
	PORTB &= ~(1 << PINB7); // UTI Power Down LOW for Power down
	
	for (unsigned char i = 0; i < 18; i++) {
		UTI_read_data[i] = getPeriod(i); // get UTI period counts
		
	}
}