/*
 * Joy2x3.c
 *
 * Created: 08-10-2015 14:34:56
 *  Author: Nelson
 */ 

#include "joy2x3.h"

#include <avr/io.h>

////////////////////////////
// [up]		[down]
// [left]	[right]	[enter]
///////////////////////////

//prototypes//
const char* decodeJoy(uint8_t codeJoy);

/************************************************************************/
/* Get joystick position                                                                     */
/************************************************************************/

char* readJoystk(void){
	uint8_t col;
	uint8_t row;
	uint8_t codeJoy;
	uint8_t swCount=0;

	JOY_PORT = 0x03;	//enable pull ups 0b00000011

	for(col=0; col <3; col++){
		JOY_DDR = 0x00;
		JOY_DDR = (0x10>>col);
		for (row=0;row<2;row++){
			if((JOY_PIN & (0x02>>row))==0){
				swCount++;
				codeJoy = (row*2+col);
			}
			
		}
	}
	if (swCount==1)return decodeJoy(codeJoy);
	return NULL;
}

/************************************************************************/
/* @decode Joystick                                                                     */
/************************************************************************/
const char* decodeJoy(uint8_t codeJoy){


	switch (codeJoy){
		case 0:
		return 	"UP";
		break;
		case 1:
		return "DOWN";
		break;
		case 2:
		return	"LEFT";
		break;
		case 3:
		return	"RIGHT";
		break;
		case 4:
		return	"ENTER";
		break;
		default:
		return NULL;
		break;
	}
	
	
}