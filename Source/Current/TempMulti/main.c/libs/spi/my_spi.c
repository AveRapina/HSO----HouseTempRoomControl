/*
 * SPI.c
 *
 * Created: 05-08-2015 15:33:47
 */ 



#include "my_spi.h"

/* define CPU frequency in Mhz here if not defined in Makefile */
#ifndef F_CPU
#define F_CPU 2000000UL
#endif

#include <avr/io.h>
/* define CPU frequency in Mhz here if not defined in Makefile */


#include "../utils/bit_tools.h"

/************************************************************************/
/*@Initialize SPI Master Device (without interrupt)                     */
/************************************************************************/
void SPI_initMaster_NO_INT (void) //CHECK WITH ARGUMENTS mode
{
	// Set MOSI, SCK as Output
	DDRB = (1<<2)|(1<<1);
	
	// Enable SPI, Set as Master
	//Prescaler: Fosc/16, 
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}

/************************************************************************/
/*@Initialize SPI Master Device (with SPI interrupt)                    */
/************************************************************************/
void SPI_initMaster_INT (void){
	
	// Set MOSI, SCK as Output
	DDRB=(1<<2)|(1<<1);
	
	// Enable SPI, Set as Master
	// Prescaler: Fosc/16, Enable Interrupts
	//The MOSI, SCK pins are as per ATMega128
	SPCR=(1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPIE);
	
}

/************************************************************************/
/* @Initialize SPI Slave Device                                                                     */
/************************************************************************/
void SPI_initSlave (void)
{
	DDRB = (1<<3);     //MISO as OUTPUT
	SPCR = (1<<SPE);   //Enable SPI
}


/************************************************************************/
/* @SPI_TRANSCEIVER                                                     */
/************************************************************************/
uint8_t SPI_transceiver (uint8_t data){
	
	// Load data into the buffer
	SPDR = data;
	
	//Wait until transmission complete
	while(!(SPSR & (1<<SPIF) ));
	
	// Return received data
	return(SPDR);
}