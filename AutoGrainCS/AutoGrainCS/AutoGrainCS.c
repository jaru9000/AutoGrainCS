/*
 * AutoGrainMoistureControl.c
 *
 * Created: 4/9/2014 8:09:45 PM
 *  Author: ruizj
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h> // for _no_op
#include <avr/sleep.h>

#include "TWI_Master.h"
#include "dht11.h"
#include "icp.h"

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
DHT11 dht;
int status;
icp_sample_t UTI_read_data[5];
//icp_sample_t sample;

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
	DDRD |= (1<<PIND3);
	PORTD |= (1<<PIND3);
	TWI_Master_Initialise();
	sleep_disable();
	
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
					 status = dht11Read(&dht);
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
	TWI_Start_Transceiver_With_Data(messageBuf,1);
	
	TWI_operation = SEND_DATA;

				if (TWI_operation == SEND_DATA)
				{ 
					// Send data to slave
					messageBuf[0] = (TWI_targetSlaveAddress<<TWI_ADR_BITS) | (FALSE<<TWI_READ_BIT);
					messageBuf[1] = status;//dht.humidity;
					messageBuf[2] = dht.temperature; 
					messageBuf[3] = UTI_read_data[0];
					messageBuf[4] = UTI_read_data[1];
					messageBuf[5] = UTI_read_data[2];
					messageBuf[6] = UTI_read_data[3];
					messageBuf[7] = UTI_read_data[4];
					TWI_Start_Transceiver_With_Data( messageBuf, messageBuf_size );
					while ( TWI_Transceiver_Busy() ); // Should wait for completion.
				 }

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
	icp_init();
	sleep_enable();
	
	//sample = icp_rx();
	UTI_read_data[0] = icp_rx();
	for (char i = 0; i < 4; i++)
	{
		//do
		//{
			UTI_read_data[i+1] = icp_rx();
		//}
		// Allow for variations in reading the capacitance values and an empty buffer (empty buffer returns full value)
		//while (UTI_read_data[i+1] <= UTI_read_data[i]+200 || UTI_read_data[i+1] >= UTI_read_data[i]-200 || UTI_read_data[i+1] > 0x3D54);
	}
	sleep_cpu();
}