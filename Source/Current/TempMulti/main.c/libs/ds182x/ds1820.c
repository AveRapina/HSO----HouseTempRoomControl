/*
 * ds1820.c
 *
 * Created: 24.08.2015 18:44:41
 *  Author: HSO
 */ 


#include "ds1820.h"
#include "../usart/usart.h"

/* define CPU frequency in Mhz here if not defined in Makefile */
#ifndef F_CPU
#define F_CPU 1600000UL
#endif



#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include "../utils/bit_tools.h"
#include "../utils/my_utils.h"


// SDA
static uint8_t DS18_readSDA(void){ return (PIN(DS18_PORT) & DS18_SDA) != 0;}
static void DS18_sdaLO(void){clrBits(DDR(DS18_PORT), DS18_SDA);}
static void DS18_sdaHI(void){ setBits(DDR(DS18_PORT), DS18_SDA);}

static void DS18_delay(void){_delay_us(2);}	







/************************************************************************/
/* @  reset ds18                                                                 */
/************************************************************************/
void DS18_reset(void){
	
	//setBits(DDR(DS18_PORT), DS18_SDA);
	DS18_sdaLO();
	_delay_us(500);
	DS18_sdaHI();
	_delay_us(500);
	DS18_sdaHI();
	
}



void DS18_writeByte(uint8_t data){
uint8_t i=0;

	for(i=0; i<8; ++i){
		DS18_sdaLO();
		DS18_delay();
		if (data &0x01) DS18_sdaHI();
		_delay_us(60);
		DS18_sdaHI();
		DS18_delay();
		data >>=1;
	}	
	
}


/************************************************************************/
/* @ one wire read                                                                     */
/************************************************************************/
uint8_t DS18_readByte(void){
uint8_t i=0;
uint8_t data=0;

	for (i=0; i<8; ++i){
		data >>=1;
		DS18_sdaLO();
		DS18_delay();
		
		DS18_sdaHI();
		_delay_us(14);
		if(DS18_readSDA()) data |=0x80; //LSB first
		
		_delay_us(45);
	}	
	
	return data;
}


double DS18_getTemp(void){
int8_t dataHigh,dataLow;
	DS18_reset();
	
	/*
	DS18_writeByte(DS18_CMD_SKIPROM); // SKIP ROM
	DS18_writeByte(DS18_CMD_CONVERTTEMP); // convert
	while(DS18_readSDA()==0);
	
	DS18_reset();
	DS18_writeByte(DS18_CMD_SKIPROM); // SKIP ROM
	DS18_writeByte(DS18_CMD_RSCRACTHPAD); // read reg
	
	dataHigh = DS18_readByte();
	dataLow= DS18_readByte();
	
	*/
	char buffer[10];
	sprintf(buffer,"%x:%x\n\r",dataHigh,dataLow);
	USART1_sendStr(buffer);
	
	
	return (double)(( dataHigh << 8 ) + dataLow ) * 0.0625;
}