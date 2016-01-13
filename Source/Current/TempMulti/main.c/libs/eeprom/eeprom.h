/*
 * eeprom.h
 *
 * Created: 07-08-2015 14:09:20
 *
 */ 

#include <avr/io.h>

/************************************************************************/
/* @defines                                                             */
/************************************************************************/
#define EEPROM_ENABLE_WRITE (1<<EEMWE)
#define EEPROM_WRITE (1<<EEWE)
#define EEPROM_READ (1<<EERE)

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/
void EEPROM_write(uint8_t addr, char data);
uint8_t EEPROM_read(uint8_t addr);

