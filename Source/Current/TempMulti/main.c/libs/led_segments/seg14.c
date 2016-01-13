/*
 * seg14.c
 *
 * Created: 08-10-2015 18:36:38
 *  Author: Nelson
 */ 

#include "../pcf857x/pcf8575.h" 

#include "seg14.h"
#include <stdio.h>
#include <avr/delay.h>



//////////////////////////////////
//        a
//       ___
//     f|\|/|b		\h |i /j
//     g1- -g2
//     e|/|\|c		/m |l \k
//		  d
///////////////////////////////////





/************************************************************************/
/*@set value to show                                                    */
/************************************************************************/


static char  actualSentence[10]; // deve chegar
static uint8_t flagNewValue;




/************************************************************************/
/* @decode the value to light up segments                                                                     */
/************************************************************************/
uint16_t bcd14Seg(char c){
	
	switch(c){
		
		case '0': return ~0b0000000000111111;
		break;
		
		case '1': return ~0b0000000000000110;
		break;
		
		case '2': return ~0b0000000011011011;
		break;
		
		case '3': return ~0b0000000011001111;
		break;
		
		case '4': return ~0b0000000011100110;
		break;
		
		case '5': return ~0b0000000011101101;
		break;
		
		case '6': return ~0b0000000011111101;
		break;
		
		case '7': return ~0b0000000000000111;
		break;
		
		case '8': return ~0b0000000011111111;
		break;
		
		case '9': return ~0b0000000011100111;
		break;
		
		case 'A': return ~0b0000000011110111; // A
		break;
		
		case 'B': return ~0b0001001010001111; // B
		break;
		
		case 'C': return ~0b0000000000111001; // C
		break;
		
		case 'D': return ~0b0001001000001111;// D
		break;
		
		case 'E': return ~0b0000000011111001;// E
		break;
		
		case 'F': return ~0b0000000011110001;// F
		break;
		
		case 'G': return ~0b0000000010111101;// G
		break;
		
		case 'H': return ~0b0000000011110110;// H
		break;
		
		case 'I': return ~0b0001001000001001;// I
		break;
		
		case 'J': return ~0b0000000000011110;// J
		break;
		
		case 'K': return ~0b0000110001110000;// K
		break;
		
		case 'L': return ~0b0000000000111000;// L
		break;
		
		case 'M': return ~0b0000010100110110;// M
		break;
		
		case 'N': return ~0b0000100100110110;// N
		break;
		
		case 'O': return ~0b0000000000111111;// O
		break;
		
		case 'P': return ~0b0000000011110011;// P
		break;
		
		case 'Q': return ~0b0000100000111111;// Q
		break;
		
		case 'R': return ~0b0000100011110011;// R
		break;
		
		case 'S': return ~0b0000000110001101;// S
		break;
		
		case 'T': return ~0b0001001000000001;// T
		break;
		
		case 'U': return ~0b0000000000111110;// U
		break;
		
		case 'V': return ~0b0010010000110000;// V
		break;
		
		case 'W': return ~0b0010100000110110;// W
		break;
		
		case 'X': return ~0b0010110100000000;// X
		break;
		
		case 'Y': return ~0b0001010100000000;// Y
		break;
		
		case 'Z': return ~0b0010010000001001;// Z
		break;
		
		// lower case
		case 'a': return ~0b0000000011011111;//a
		break;
		
		case 'b': return ~0b0000100001111000;//b
		break;
		
		case 'c': return ~0b0000000011011000;//c
		break;
		
		case 'd': return ~0b0010000010001110;//d
		break;
		
		case 'e': return ~0b0000000001111001;//e
		break;
		
		case 'f': return ~0b0000000001110001;//f
		break;
		
		case 'g': return ~0b0000000110001111;//g
		break;
		
		case 'h': return ~0b0000000011110100;//h
		break;
		
		case 'i': return ~0b0001000000000000;//i
		break;
		
		case 'j': return ~0b0000000000001110;//j
		break;
		
		case 'k': return ~0b0001111000000000;//k
		break;
		
		case 'l': return ~0b0001001000000000;//l
		break;
		
		case 'm': return ~0b0001000011010100;//m
		break;
		
		case 'n': return ~0b0000000011010100;//n
		break;
		
		case 'o': return ~0b0000000011011100;//o
		break;
		
		case 'p': return ~0b0000010001110001;//p
		break;
		
		case 'q': return ~0b0000100011100011;//q
		break;
		
		case 'r': return ~0b0000000001010000;//r
		break;
		
		case 's': return ~0b0000000110001101;//S
		break;
		
		case 't': return ~0b0000000001111000;//t
		break;
		
		case 'u': return ~0b0000000000011100;//u
		break;
		
		case 'v': return ~0b0010000000010000;//v
		break;
		
		case 'w': return ~0b0010100000010100;//w
		break;
		
		case 'x': return ~0b0010110100000000;//x
		break;
		
		case 'y': return ~0b0000001010001110;//y
		break;
		
		case 'z': return ~0b0010010000001001;// Z
		break;
		
		case '.': return ~0b1100000000000000;//.
		break;
		
		case '*': return ~0b0011111100000000;//*
		break;
		
		case '+': return ~0b0001001011000000;//+
		break;
		
		case '-': return ~0b0000000011000000;//-
		break;
		
		case '=': return ~0b0000000011001000;//=
		break;
		
		case '(': return ~0b0000000000111001;//(
		break;
		
		case ')': return ~0b0000000000001111;//)
		break;
		
		case '/': return ~0b0010010000000000;// /
		break;
		
		case '\\': return ~0b0000100100000000;// \
		break;
		
		case '%': return ~0b0010010000100100;//%
		break;
		
		case ',': return ~0b0010000000000000;//,
		break;
		
		case '\'': return ~0b0000001000000000;//'
		break;
		
		case '_': return ~0b0000000000001000;//_
		break;
		
		case '$': return ~0b0001001011101101;//$
		break;

		case '&': return ~0b0000110101011001;//&
		break;
		
		case '@': return ~0b0000001010111011;//@
		break;
		
		default: return  ~0b0000000000000000; // all off
		break;
		
	}
	
}


/************************************************************************/
/* @update new value to show                                                                     */
/************************************************************************/
void SEG14_setNewValue(char * sentence){
	
	sprintf(actualSentence,sentence);
	flagNewValue++;
}



/************************************************************************/
/* @update next segment                                                                     */
/************************************************************************/
void SEG14_nextSegment(void){
	static colSegment;

	colSegment++;
	if (colSegment >= N_SEG14) colSegment=0;
	uint16_t colValue = ~(1<<colSegment);// chece to simulate 
	//uint16_t colValue = (1<<colSegment);
	uint16_t rowValue =  bcd14Seg(actualSentence[colSegment]);

	
	// depois faz-se, testa esta primeiro,
	
	
	// set all segments off
	PCF16_write(COL_14_ADD,0xffff);
	_delay_us(100);
	// set the new value
	PCF16_write(ROW_14_ADD,rowValue);
	// ligh up the segment
	PCF16_write(COL_14_ADD,colValue);
	
	
	
	
}


