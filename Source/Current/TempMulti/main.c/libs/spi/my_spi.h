/*
 * SPI.h
 *
 * Created: 05-08-2015 15:19:14
 *  Author: Nelson
 */ 
#include <avr/io.h>


// prototypes
void SPI_initMaster_NO_INT (void);
void SPI_initMaster_INT (void);
void SPI_initSlave (void);
uint8_t SPI_transceiver (uint8_t data);