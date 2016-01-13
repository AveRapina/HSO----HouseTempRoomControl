/*
 * eeprom.c
 *
 * Created: 07-08-2015 14:13:27
 *  Author: HSO
 */ 
/* define CPU frequency in Mhz here if not defined in Makefile */
#ifndef F_CPU
#define F_CPU 2000000UL
#endif

#include <avr/io.h>
#include "eeprom.h"

#ifndef F_CPU
#define F_CPU 2000000UL
#endif

/************************************************************************/
/* @EEPROM                  Always use OUT or SBI to set EERE in EECR.  */  //CHECK
/************************************************************************/

/************************************************************************/
/* @EEPROM WRITE                                                        */
/************************************************************************/
void EEPROM_write(uint8_t addr, char data){
	/* Wait for completion of previous write */
	while(EECR & (1<<EEWE));
	/* Set up address and data registers */
	EEAR = addr;
	EEDR = data;
	/* Write logical one to EEMWE */
	EECR |= EEPROM_ENABLE_WRITE; //
	/* Start eeprom write by setting EEWE */
	EECR |= EEPROM_WRITE;
}


/************************************************************************/
/* @EEPROM READ                                                         */
/************************************************************************/
uint8_t EEPROM_read(uint8_t addr){
	/* Wait for completion of previous write */
	while(EECR & (1<<EEWE));
	/* Set up address register */
	EEAR = addr;
	/* Start eeprom read by writing EERE */
	EECR = EEPROM_READ;
	/* Return data from data register */
	return EEDR;
}
