/*
 * AutoGrainMoistureControl.c
 *
 * Created: 4/9/2014 8:09:45 PM
 *  Author: ruizj
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h> // for _no_op

#include "TWI_Master.h"

// Addr. and FT-X commands
#define TWI_GEN_CALL	0x00
#define TWI_targetSlaveAddress	0x22 // Same as general call because there is only one device on the line

// Transmission States
#define SEND_DATA             0x01
#define REQUEST_DATA          0x02
#define READ_DATA_FROM_BUFFER 0x03
#define SUCCESSFUL_TRANSMIT	  0x04

#define messageBuf_size		  0x08

// Other definitions
unsigned char size_of_buffer_out;
unsigned char USB_data;
unsigned char messageBuf[messageBuf_size];
unsigned char TWI_operation;

void Send_Data_to_LabVIEW();

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
	TWI_Master_Initialise();
	//__enable_interrupt();
	sei();

    while(1)
    {
      // Check if Send_Data command is in from LabVIEW
	  messageBuf[0] = (TWI_targetSlaveAddress<<TWI_ADR_BITS) | (TRUE<<TWI_READ_BIT);
      TWI_Start_Transceiver_With_Data(messageBuf,2);
	  TWI_Get_Data_From_Transceiver( messageBuf, 2 ); // Read I2C's data
      
     
	      // Check if the TWI Transceiver has completed an operation.
		      // Check if the last operation was successful
		      if ( TWI_statusReg.lastTransOK )
		      {
			      
				      USB_data = messageBuf[1];        // Store data. 

				   if (USB_data == 0xFF)
				   {
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
	TWI_Start_Transceiver_With_Data(messageBuf,1);
	
	TWI_operation = SEND_DATA;

				if (TWI_operation == SEND_DATA)
				{ 
					// Send data to slave
					messageBuf[0] = (TWI_targetSlaveAddress<<TWI_ADR_BITS) | (FALSE<<TWI_READ_BIT);
					messageBuf[1] = 2;
					messageBuf[2] = 3;
					messageBuf[3] = 4;
					messageBuf[4] = 5;
					messageBuf[5] = 6;
					messageBuf[6] = 7;
					messageBuf[7] = 8;
					TWI_Start_Transceiver_With_Data( messageBuf, 7 );
					while ( TWI_Transceiver_Busy() ); // Should wait for completion.
				 }

			if (!TWI_statusReg.lastTransOK )  // Got an error during the last transmission
			{
				// Use TWI status information to determine cause of failure and take appropriate actions.
				TWI_Act_On_Failure_In_Last_Transmission( TWI_Get_State_Info( ) );
			}

				USB_data = 0x00;
		
	
}