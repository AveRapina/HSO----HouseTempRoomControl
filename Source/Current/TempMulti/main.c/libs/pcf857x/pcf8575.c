/*
 * pcf8575.c
 *
 * Created: 08-10-2015 18:28:32
 *  Author: Nelson
 */ 

#include "pcf8575.h"


/* define CPU frequency in Mhz here if not defined in Makefile */
#ifndef F_CPU
#define F_CPU 2000000UL
#endif



#include <avr/io.h>
#include <util/delay.h>
#include "../utils/bit_tools.h"

#include "../i2c/twi.h"

volatile uint8_t pcfPinVal16;


uint16_t PCF16_writePin(uint8_t addr, uint16_t pin, uint16_t val){
	uint16_t b = 0;
	
	b= pcfPinVal16;
	// put zero or one in bit
	b = (val |= 0) ? (b | (1 << pin)) :(b & -(1 <<pin));
	// write
	PCF16_write(addr,b);
	return 0;
	
	
	
}



/************************************************************************/
/*                                                                      */
/************************************************************************/
int16_t PCF16_readPin(uint8_t addr, uint16_t pin){
	int16_t data =  -1;
	
	data = PCF16_read(addr);
	
	if(data != -1) data = (data >> pin) &0x01;
	
	return data;
}


/************************************************************************/
/* @ pcf                                                                      */
/************************************************************************/
uint16_t PCF16_read(uint8_t addr){
	uint16_t data=0;
	
	I2C_start();
	I2C_sendAddr(addr+1);
	data=I2C_receiveByte_ACK() <<8;
	data=I2C_receiveByte_NACK();
	I2C_stop();
	return data;
}


/************************************************************************/
/* @ pcf                                                                      */
/************************************************************************/
void PCF16_write(uint8_t addr, uint16_t data){

	// save values to array by defaults
	pcfPinVal16 = data;
	
	I2C_start();
	I2C_sendAddr(addr);
	I2C_sendByte(data );
	I2C_sendByte(data >>8);
	
	I2C_stop();
}


// 16 bits