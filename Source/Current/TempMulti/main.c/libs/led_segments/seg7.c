/*
 * _7seg.c
 *
 * Created: 06-10-2015 20:37:46
 *  Author: Nelson
 */ 

#include "../pcf857x/pcf857x.h" // CHECK, rename lib to pcf8574

#include "seg7.h"
#include <stdio.h>
#include <avr/delay.h>



//////////////////////////////////
//        a
//       __
//     f|  |b
//      |__|
//      | g|
//     e|__|c
//       d
/////////////////////////////////// 

static uint16_t actualValue;
static uint8_t flagNewValue;

/************************************************************************/
/* @decode the value to light up segments                                                                     */
/************************************************************************/
static uint8_t bcdSevenSeg(uint8_t val){
	
	switch(val){
		
		case 0x00: return ~0b00111111;
		break;
		
		case 0x01: return ~0b00000110;
		break;
		
		case 0x02: return ~0b01011011;
		break;
		
		case 0x03: return ~0b01001111;
		break;
		
		case 0x04: return ~0b01100110;
		break;
		
		case 0x05: return ~0b01101101;
		break;
		
		case 0x06: return ~0b01111101;
		break;
		
		case 0x07: return ~0b00000111;
		break;
		
 		case 0x08: return ~0b01111111;
 		break;
		 
		case 0x09: return ~0b01100111;
		break;
		
		case 0xA: return ~0b01110111; // A
 		break;
		
		case 0xB: return ~0b01111100; // b
		break;
		
		case 0xC: return ~0b00111001; // C
		break;
		
		case 0xD: return ~0b01011110;// d
		break;
		
		case 0xE: return ~0b01111001;// E
		break;
		
		case 0xF: return ~0b01110001;// F
		break;
		
		
		default: return NULL;
		break;
	
	}
	
}

/************************************************************************/
/* @update new value to show                                                                     */
/************************************************************************/
void SEG_setNewValue(uint16_t val){
	
	actualValue=val;
	flagNewValue++;
}


/************************************************************************/
/* @extart the digit from the  global variable                                                                     */
/************************************************************************/
static uint8_t getDecimalValue(uint8_t digit){
	// divir por 10 e obter digito // 65535
uint8_t temp;
uint16_t remainder;
static uint8_t digitValues[5];

	// perform division if value changes
	if(flagNewValue){
		// perform division and put in the array
		temp = actualValue/10000;
		remainder = actualValue%10000;
		digitValues[4] = temp;
		
		temp = remainder/1000;
		remainder = remainder%1000;
		digitValues[3] = temp;
		
		temp = remainder/100;
		remainder = remainder%100;
		digitValues[2] = temp;
		
		
		temp = remainder/10;
		remainder = remainder%10;
		digitValues[1] = temp;
		
		
		digitValues[0] = remainder;

		// clear flag
		flagNewValue=0;
	}
	
	// return digit
	return digitValues[digit];
	
	
	
	
}



/************************************************************************/
/* @update next segment                                                                     */
/************************************************************************/
void SEG_nextSegment(void){
static colSegment;

	colSegment++;
	if (colSegment >= N_SEG7) colSegment=0;
	uint8_t colValue = ~(1<<colSegment);
	uint8_t rowSegment = getDecimalValue(colSegment);
	uint8_t segVal= bcdSevenSeg(rowSegment);
	
	// set all segments off
	PCF_write(COL_ADD,0xff);
	_delay_us(100);
	// set the new value
	PCF_write(ROW_ADD,segVal);
	// ligh up the segment
	PCF_write(COL_ADD,colValue);
	
		
	
}






/************************************************************************/
/* set new value to show                                                */
/************************************************************************/
void SEG_setValue(uint8_t addr,int val){
	
	PCF_write(addr,bcdSevenSeg(val));
	
}
