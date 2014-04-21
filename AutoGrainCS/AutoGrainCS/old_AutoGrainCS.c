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
#define TWI_targetSlaveAddress	0x22 

// Transmission States
#define SEND_DATA             0x01
#define REQUEST_DATA          0x02
#define READ_DATA_FROM_BUFFER 0x03
#define SUCCESSFUL_TRANSMIT	  0x04

#define messageBuf_size		  0x11
#define TWI_init_messageBuf_size 0x02

// Other definitions
unsigned char size_of_buffer_out;
unsigned char USB_data;
unsigned char messageBuf[messageBuf_size];
unsigned char TWI_operation;
DHT11 dht;
int status;
//icp_sample_t UTI_read_data[5];
unsigned int utiCount;
unsigned int UTI_read_data[6];
icp_sample_t period;
icp_sample_t period_old;


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
	
	//__enable_interrupt();
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
	
	TWI_operation = SEND_DATA;

				if (TWI_operation == SEND_DATA)
				{ 
					// Send data to slave
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
/*void Get_UTI_Data()
{	DDRD |= (1 << PIND3);
	for (int i=0; i < 3; i++) UTI_read_data[i] = 0;
	//icp_init();
	//sleep_enable();
	DDRD &= ~(1 << PIND4);
	PORTD &= ~(1 << PIND4);
	//sample = icp_rx();
	//UTI_read_data[0] = icp_rx();
	PORTB |= (1 << PINB7); // UTI Power Down High for Active
	for (unsigned char i = 0; i < 6; i++)
	{
		utiCount = 0;
		while (bit_is_clear(PIND, PIND4)) {
			PORTD |= (1 << PIND3);
			_NOP();
		} 
		
		while(bit_is_set(PIND, PIND4)) {
			utiCount++;
			//utiCount = i;
		}
		UTI_read_data[i] = utiCount;
		
		//do
		//{
			//UTI_read_data[i+1] = icp_rx();
		//}
		// Allow for variations in reading the capacitance values and an empty buffer (empty buffer returns full value)
		//while (UTI_read_data[i+1] <= UTI_read_data[i]+200 || UTI_read_data[i+1] >= UTI_read_data[i]-200 || UTI_read_data[i+1] > 0x3D54);
		
	}
	//PORTB &= ~(1 << PINB7); // UTI Power Down LOW for Power down
	PORTB &= ~(1 << PIND3);
	//sleep_cpu();
} */

// Function that gets data from UTI
void Get_UTI_Data()
{	
	PORTB |= (1 << PINB7); // UTI Power Down High for Active
	icp_init();
	period = icp_rx();
	period_old = period;
	
	for (unsigned char i = 0; i < 5; i++)
	{
		while (period_old == period || period == 0xFFFF || period == 0x0000) {
			period = icp_rx();
			//_NOP();
		}
		
		UTI_read_data[i] = period;
		period_old = period;
		
		
		
	}
	PORTB &= ~(1 << PINB7); // UTI Power Down LOW for Power down
}