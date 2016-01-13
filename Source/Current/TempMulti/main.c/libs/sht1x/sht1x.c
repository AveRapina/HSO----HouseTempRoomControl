/*
* SHT11.c
*
* Created: 27-07-2015 17:32:57
* 
*/
// ctrl+D indent code
#include "sht1x.h"

/* define CPU frequency in Mhz here if not defined in Makefile */
#ifndef F_CPU
#define F_CPU 1600000UL
#endif



#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include "../utils/bit_tools.h"


//debug
#include "../usart/usart.h"
char dummy [30];




//CLK
static void SHT11_sckHI(void){setBits(PORT(SHT11_PORT), SHT11_SCL);}
static void SHT11_sckLO(void){clrBits(PORT(SHT11_PORT), SHT11_SCL);}
// SDA
static uint8_t SHT11_readSDA(void){ return (PIN(SHT11_PORT) & SHT11_SDA) != 0;}
static void SHT11_sdaLO(void){setBits(DDR(SHT11_PORT), SHT11_SDA);}
static void SHT11_sdaHI(void){ clrBits(DDR(SHT11_PORT), SHT11_SDA);}
	
static void SHT11_delay(void){_delay_us(2);}	
static void SHT11_sckPulse(void) {SHT11_sckHI(); SHT11_delay(); SHT11_sckLO();}


static uint8_t crc_value;


/************************************************************************/
/* @HARD POWER UP                                                                     */
/************************************************************************/
void SHT11_hardPowerOn(void){
	SHT11_POWER_UP;
	SHT11_init();
}

/************************************************************************/
/* @HARD POWER UP                                                                     */
/************************************************************************/
void SHT11_hardPowerOFF(void){
	SHT11_POWER_DOWN;
}

/************************************************************************/
/* @check CRC                                                                     */
/************************************************************************/
void SHT11_crc8(uint8_t b){
	for (uint8_t i = 0; i < 8; ++i) {
		if ((crc_value ^ b) & 0x80) {
			crc_value <<= 1;
			crc_value ^= 0x31;
		} else
		crc_value <<= 1;
		b <<= 1;
	}
}


/************************************************************************/
/* @wait com free                                                                     */
/************************************************************************/
void SHT11_waitComFree(void){
int16_t delay=0;	
	
	SHT11_delay();
	
	for (delay=0; delay <30000; delay++){ //300ms max
		if(SHT11_ready!=0)break;
		_delay_us(10);
	}
	
}


/************************************************************************/
/* @init Sht                                                                      */
/************************************************************************/
void SHT11_init(void){
	
	
	
	clrBits(PORT(SHT11_PORT), SHT11_SCL | SHT11_SDA);	// SCK output low, SDA input/high
	setBits(DDR(SHT11_PORT),  SHT11_SCL);
	clrBits(DDR(SHT11_PORT),  SHT11_SDA);
	SHT11_delay();
	
	SHT11_resetCom();
	SHT11_transmissionStart();
	SHT11_writeByte(SHT11_CMD_RESET);
	_delay_ms(11);
	
	// By default start with 12/14 resolution

	//SHT11_transmissionStart();
	//SHT11_writeData(SHT11_CMD_WSTAT);
	//SHT11_writeData(SHT11_LOWRES);
}


/************************************************************************/
/* @@function reset SHT11                                                                    */
/************************************************************************/
/*************************************************************************/
// SCK	  _   _   _   _      _
//      _| |_| |_| |_| |_...|9|_						
// SDA  _________________    ___	
//                       ...    |                                         
/************************************************************************/
void SHT11_resetCom(void){
uint8_t i=0;
	SHT11_sdaHI();
	for (i=0; i<10; ++i){ SHT11_sckPulse(); SHT11_delay();} // toogle sck 9 or more times
		
}



/************************************************************************/
/* @SHT11 transmission Start                                                                     */
/************************************************************************/
/*************************************************************************/
// SCK	  _   _  
//      _| |_| |_											
// SDA  __     __											
//        |___|  |_                                         
/************************************************************************/
void SHT11_transmissionStart(void){
			
	// Start sequence
	SHT11_sdaHI(); SHT11_delay();
	SHT11_sckHI(); SHT11_delay();
	SHT11_sdaLO(); SHT11_delay();
	SHT11_sckLO(); SHT11_delay();
	SHT11_sckHI(); SHT11_delay();
	SHT11_sdaHI(); SHT11_delay();
	SHT11_sckLO(); SHT11_delay();
			
}



/************************************************************************/
/* @function write comand to sht     //OK                                                                */
/************************************************************************/
uint8_t SHT11_writeByte(uint8_t data){
uint8_t i=0;
uint8_t ack=0;

	for(i=0; i<8; i++){
		if(data&0x80)SHT11_sdaHI();
		else SHT11_sdaLO();
		// issue clock
		data <<=1;
		SHT11_delay();
		SHT11_sckPulse();;
		
	}
	
	SHT11_sdaHI();
	_delay_us(1);
	// get ack value
	ack=SHT11_readSDA();
	// Issue clck
	SHT11_sckPulse();
	
	return ack;
	
}


//CHECK THIS!
/************************************************************************/
/* @chck if sht has converted data                                                                     */
/************************************************************************/
uint8_t SHT11_ready(void){

	return SHT11_readSDA() == 0; // true on ready	
}


/************************************************************************/
/* @receive data                                                                     */
/************************************************************************/
uint8_t SHT11_readByte(void){
uint8_t data=0;
uint8_t i;

	//-- 1. lets shift MSB out
	for (i=0; i<8; i++){
		data <<=1;
		if(SHT11_readSDA() >0) data |=1;
		// zero by default
		SHT11_sckPulse();
	}
	
	//Low ack
	SHT11_sdaLO();
	_delay_us(2);
	// issue clock
	SHT11_sckPulse();
	// restore SDA hi
	SHT11_sdaHI();
	return data;
	
}


/************************************************************************/
/* @sht read CRC                                                                     */
/************************************************************************/
uint8_t SHT11_readCRC(void){
uint8_t crc;
uint8_t i=0;
	
	//LSB for CRC fisrt
	for(i=0; i<8; ++i){
		crc >>=1;
		if(SHT11_readSDA()) crc |=1;
		
		SHT11_sckPulse();
	}
	
	
	// HI ack
	SHT11_sdaHI();
	SHT11_delay();
	SHT11_sckPulse();
	return crc;
	
}



/************************************************************************/
/* @ issue convestion                                                       */
/************************************************************************/
int8_t SHT11_startMeasure(uint8_t cmd){
	uint8_t ack;

	SHT11_transmissionStart();
	ack = SHT11_writeByte(cmd);
	return ack;
}



/************************************************************************/
/* @read sht values                                                                     */
/************************************************************************/
int16_t SHT11_result(void){
int16_t v=0;
uint8_t crc=0;
	
	if (!SHT11_ready())	return SHT11_UNAVAIL;
	v = SHT11_readByte() << 8; v |= SHT11_readByte();
	
	//crc = SHT11_readCRC();
	//if (crc != crc_value) return SHT11_CRC_FAIL;
	
	return v;
}


/************************************************************************/
/* @sht read temp                                                                     */
/************************************************************************/
int16_t SHT11_readTemp(uint8_t conv){
	
	int16_t v = SHT11_result();
	// DEBUG
	sprintf(dummy,"%x \n\r",v);
	USART0_sendStr(dummy);
	
	
	if (sht11_valid(v)) {
		if(conv == SHT11_CONVERT_READ){
			#if SHT11_LOWRES
			v = v * 4 - 4000;
			#else
			v -= 4000;
			#endif
		}
		else return v;
	}
	return v;
	
}


/************************************************************************/
/* @sht read RH                                                                     */
/************************************************************************/
int16_t SHT11_readRH(uint8_t mode){
	
	 int16_t v = SHT11_result();
	 // DEBUG
	 sprintf(dummy,"%x \n\r",v);
	 USART0_sendStr(dummy);
	 
	 if (sht11_valid(v)) {
		 if(mode == SHT11_CONVERT_READ){
			#if SHT11_LOWRES
			// inspired by Werner Hoch, modified for low resolution mode
			const int32_t C1 = (int32_t)(-4.0 * 100);
			const int32_t C2 = (int32_t)(0.648 * 100 * (1L<<24));
			const int32_t C3 = (int32_t)(-7.2e-4 * 100 * (1L<<24));
			v = (int16_t)((((C3 * v + C2) >> 7) * v + (1L<<16)) >> 17) + C1;
			#else
			// inspired by Werner Hoch
			const int32_t C1 = (int32_t)(-4.0 * 100);
			const int32_t C2 = (int32_t)(0.0405 * 100 * (1L<<28));
			const int32_t C3 = (int32_t)(-2.8e-6 * 100 * (1L<<30));
			v = (int16_t)((((((C3 * v) >> 2) + C2) >> 11) * v + (1L<<16)) >> 17) + C1;
			#endif
		 }
		 else return v;
	 }
	 return v;
	
}

/************************************************************************/
/* @read rh compensated                                                                     */
/************************************************************************/
void  SHT11_calValues(int16_t temp, int16_t humid, float *tc, float *rhlin, float *rhtrue){
float rh;	
	 
	 rh= (float) humid;
	 
	 #if SHT11_LOWRES
	 *tc = (float) temp*4 - 40.0;
	 *rhlin = -4 +(rh*0.648) +(rh*rh*0.0072);
	 *rhtrue = ((*tc -25.0)*(0.01 +(0.00128*rh)))**rhlin; 
	 #else
	 *tc = (float) temp*0.01 - 40.0;
	 *rhlin = -4 +(rh*0.0405) +(rh*rh*0.0000028);
	 *rhtrue = ((*tc -25.0)*(0.01 +(0.00008*rh)))**rhlin;
	 #endif
	 
}


	