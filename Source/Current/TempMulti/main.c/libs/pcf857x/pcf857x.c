#include "pcf857x.h"


/* define CPU frequency in Mhz here if not defined in Makefile */
#ifndef F_CPU
#define F_CPU 2000000UL
#endif

//Slave addresses must be 0b0100 0000 and then 0b0100 0010 slave address must be rotated by 1, last bit is read(1)/write(0) command

#include <avr/io.h>
#include <util/delay.h>
#include "../utils/bit_tools.h"

#include "../i2c/twi.h"

volatile uint8_t pcfPinVal;


int8_t PCF_writePin(uint8_t addr, uint8_t pin, uint8_t val){
	uint8_t b = 0;
	
	b= pcfPinVal;
	// put zero or one in bit
	b = (val |= 0) ? (b | (1 << pin)) :(b & -(1 <<pin));
	// write
	PCF_write(addr,b);
	return 0;
	
	
	
}



/************************************************************************/
/*                                                                      */
/************************************************************************/
int8_t PCF_readPin(uint8_t addr, uint8_t pin){
	int8_t data =  -1;
	
	data = PCF_read(addr);
	if(data != -1) data = (data >> pin) &0x01;
	
	return data;
}


/************************************************************************/
/* @ pcf                                                                      */
/************************************************************************/
uint8_t PCF_read(uint8_t addr){
uint8_t data=0;
	
	I2C_start();
	I2C_sendAddr(addr+1);
	data=I2C_receiveByte_NACK();
	I2C_stop();
	return data;
}


/************************************************************************/
/* @ pcf                                                                      */
/************************************************************************/
void PCF_write(uint8_t addr, uint8_t data){

	// save values to array by defaults
	pcfPinVal = data;
	
	I2C_start();
	I2C_sendAddr(addr);
	I2C_sendByte(data);
	I2C_stop();
}


