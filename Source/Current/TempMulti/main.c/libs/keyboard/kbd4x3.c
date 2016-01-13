/*
 * kbd4x3.c
 *
 * Created: 06-10-2015 19:36:11
 *  Author: Nelson
 */ 

#include "kbd4x3.h"


#include <avr/io.h>



// local prototipe
char decodekey(uint8_t key);

char decodeKey(uint8_t keyCode){
	
	if(keyCode <9) return keyCode +'1';
	else {
		switch(keyCode){
			case 9:
			return '*';
			break;
			
			case 10:
			return '0';
			break;
			
			case 11 :
			return '#';
			break;
			
			default:
			return NULL;
			break;
		}
	}
	
}


char KBD_read(void){
	
	// set the pull ups in row on
	KEYPAD_PORT =0x0f;
	
	uint8_t keyCount=0;
	uint8_t keyCode =0xff;
	// loop columns
	for (uint8_t col=0; col <3; col++){
		KEYPAD_DDR &=~(0x7F);
		KEYPAD_DDR |=(0x40 >>col);
		
		for(uint8_t row=0; row <4; row++){
			if((KEYPAD_PIN &(0x08 >>row))==0){
				keyCount ++;
				keyCode =(row*3+col); // (00)(01)(02)
			}
		}
	}
	
	// decode code
	if(keyCount == 1){
		return decodeKey(keyCode);
		
	}
	
	return NULL;
	
}