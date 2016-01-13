
#include "ade77xx.h"


/* define CPU frequency in Mhz here if not defined in Makefile */
#ifndef F_CPU
#define F_CPU 2000000UL
#endif

#include "../spi/my_spi.h"
#include "../utils/my_utils.h"

static void ADE_select(void){setBits(PORT(ADE_CS_PORT), ADE_CS_PIN);}
static void ADE_release(void){clrBits(PORT(ADE_CS_PORT), ADE_CS_PIN);}

/************************************************************************/
/* @read ade one byte                                                                     */
/************************************************************************/
uint8_t ADE7753_read8(uint8_t addr){
uint8_t rData;

	ADE_select();

	// Send address first
	SPI_transceiver(addr);
	
	_delay_us(4);

	// Send dummy data and receive data
	rData=SPI_transceiver(0x00);
	
	ADE_release();
	
	return(rData);
}


/************************************************************************/
/* @ read 2 bytes ade                                                                     */
/************************************************************************/
uint16_t ADE7753_read16(uint8_t addr){
uint16_t rData;
	
	ADE_select();
	
	// Send Address first
	SPI_transceiver(addr);
	_delay_us(4);

	// Send Dummy data and receive data
	rData=SPI_transceiver(0x00) <<8;
	// delay???
	NOP();
	rData|=SPI_transceiver(0x00);
	
	ADE_release();
	return rData;
}


/************************************************************************/
/*                                                                      */
/************************************************************************/
uint32_t ADE7753_read24(uint8_t addr){
uint32_t rData=0;
	
	ADE_select();
	
	// Send Address first
	SPI_transceiver(addr);
	_delay_us(4);
	
	// Send Dummy data and receive data
	rData |=(uint32_t) SPI_transceiver(0x00)<<16;
	// delay 1u???
	NOP();
	rData|=(uint32_t)SPI_transceiver(0x00) <<8;
	// delay 1u???
	NOP();
	rData |=(uint32_t)SPI_transceiver(0x00);

	ADE_release();
	
	return rData;
}



/************************************************************************/
/*                                                                      */
/************************************************************************/
void ADE7753_write8(uint8_t addr,uint8_t wData){
	// Write Mode
	addr |= 0x80; // mask
	
	// SELECT CHIP
	ADE_select();
	
	// Send address first
	SPI_transceiver(addr);
	_delay_us(4);
	
	// Send Data
	SPI_transceiver(wData);
	
	ADE_release();
}


	
/************************************************************************/
/*                                                                      */
/************************************************************************/
void ADE7753_write16(uint8_t addr,uint16_t wData){
	uint8_t data2=0,data1=0;
	//split data
	data1 = (uint8_t)wData;
	data2 = (uint8_t)(wData>>8);
	//==============================================
	// Write Mode
	addr |= 0x80;
	
	ADE_select();
	
	// Send address first
	SPI_transceiver(addr);
	_delay_us(4);
	
	// Send Data, MSB first
	SPI_transceiver(data2);
	NOP();
	// Send Data, LSB
	SPI_transceiver(data1);
	
	ADE_release();
}


/************************************************************************/
/*                                                                      */
/************************************************************************/
uint32_t ADE_getVRMS(void){
	// Read RSTSTATUS to clear all interrupts
	ADE7753_read16(RSTSTATUS);
	// wait Zero-Crossing
	while(!( ADE7753_read16(STATUS) & ZX));
	// return VRMS Value
	return ADE7753_read24(VRMS);
}



/************************************************************************/
/*                                                                      */
/************************************************************************/
uint32_t ADE_getIRMS(void){
	// Read RSTSTATUS to clear all interrupts
	ADE7753_read16(RSTSTATUS);
	// wait Zero-Crossing
	while(!( ADE7753_read16(STATUS) & ZX));
	// return VRMS Value
	return ADE7753_read24(IRMS);
}


/************************************************************************/
/*                                                                      */
/************************************************************************/
uint32_t ADE_getVA(void){
	// Read RSTSTATUS to clear all interrupts
	ADE7753_read16(RSTSTATUS);
	// wait Zero-Crossing
	while(!( ADE7753_read16(STATUS) & ZX));
	// return WATTS
	return ADE7753_read24(VAENERGY);
}



/************************************************************************/
/*                                                                      */
/************************************************************************/
uint32_t ADE_getWATT(void){
	// Read RSTSTATUS to clear all interrupts
	ADE7753_read16(RSTSTATUS);
	// wait Zero-Crossing
	while(!( ADE7753_read16(STATUS) & ZX));
	// return WATTS
	return ADE7753_read24(AENERGY);
}

/************************************************************************/
/* @get ade period                                                                     */
/************************************************************************/
uint32_t ADE_getPERIOD(void){
	// Read RSTSTATUS to clear all interrupts
	ADE7753_read16(RSTSTATUS);
	// wait Zero-Crossing
	while(!( ADE7753_read16(STATUS) & ZX));
	// return WATTS
	return ADE7753_read24(PERIOD);
}

