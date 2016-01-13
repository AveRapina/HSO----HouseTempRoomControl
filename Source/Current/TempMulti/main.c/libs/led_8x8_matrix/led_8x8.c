/*
 * led_8x8.c
 *
 * Created: 12-10-2015 12:19:18
 *  Author: Nelson
 */ 
#ifndef F_CPU
#define F_CPU 16000000
#endif

#include "led_8x8.h"
#include <avr/io.h>
#include <stdio.h>

#include "../utils/my_utils.h"
#include "../max7219/max7219.h" 


//////////////////////////////////
//    C0  C1 C2 C3 C4 C5 C6 C7 
//  R0 O  O  O  O  O  O  O  O
//  R1 O  O  O  O  O  O  O  O	 
//  R2 O  O  O  O  O  O  O  O 
//  R3 O  O  O  O  O  O  O  O
//	R4 O  O  O  O  O  O  O  O
//	R5 O  O  O  O  O  O  O  O
//	R6 O  O  O  O  O  O  O  O
//	R7 O  O  O  O  O  O  O  O 
///////////////////////////////////


// trial char
const char letter0[8]={
	
	0b11111111,
	0b10000011,
	0b10000101,
	0b10001001,
	0b10010001,
	0b10100001,
	0b11000001,
	0b11111111,
};

const char letter1[8]={
	
	0b00010000,
	0b00110000,
	0b01010000,
	0b00010000,
	0b00010000,
	0b00010000,
	0b00010000,
	0b01111100,
};


const char letter2[8]={
	
	0b01111100,
	0b01000010,
	0b00000010,
	0b00111110,
	0b01000000,
	0b01000000,
	0b01000000,
	0b01111110,
};

const char letter3[8]={
	
	0b01111100,
	0b01000010,
	0b00000010,
	0b00011100,
	0b00000010,
	0b00000010,
	0b00000010,
	0b01111100,
};


const char letter4[8]={
	
	0b00000100,
	0b00001100,
	0b00010100,
	0b00100100,
	0b01000100,
	0b01111110,
	0b00000100,
	0b00000100,
};

const char letter5[8]={
	
	0b11111111,
	0b10000000,
	0b10000000,
	0b11111110,
	0b00000001,
	0b00000001,
	0b00000001,
	0b11111110,
};

const char letter6[8]={
	
	0b11111111,
	0b10000000,
	0b10000000,
	0b11111111,
	0b10000001,
	0b10000001,
	0b10000001,
	0b11111111,
};

const char letter7[8]={
	
	0b01111111,
	0b00000001,
	0b00000010,
	0b00000100,
	0b00001000,
	0b00010000,
	0b00100000,
	0b01000000,
};

const char letter8[8]={
	
	0b11111111,
	0b10000001,
	0b10000001,
	0b11111111,
	0b10000001,
	0b10000001,
	0b10000001,
	0b11111111,
};

const char letter9[8]={
	
	0b11111111,
	0b10000001,
	0b10000001,
	0b11111111,
	0b00000001,
	0b00000001,
	0b00000001,
	0b00000001,
};

const char letterA[8]={
	
	0b11111111,
	0b10000001,
	0b10000001,
	0b11111111,
	0b10000001,
	0b10000001,
	0b10000001,
	0b10000001,
};

const char letterB[8]={
	
	0b11111110,
	0b10000001,
	0b10000001,
	0b11111110,
	0b10000001,
	0b10000001,
	0b10000001,
	0b11111110,
};

const char letterC[8]={
	
	0b11111111,
	0b10000000,
	0b10000000,
	0b10000000,
	0b10000000,
	0b10000000,
	0b10000000,
	0b11111111,
};

const char letterD[8]={
	
	0b11111100,
	0b10000010,
	0b10000001,
	0b10000001,
	0b10000001,
	0b10000001,
	0b10000010,
	0b11111100,
};

const char letterE[8]={
	
	0b11111111,
	0b10000000,
	0b10000000,
	0b11111110,
	0b10000000,
	0b10000000,
	0b10000000,
	0b11111111,
};

const char letterF[8]={
	
	0b11111111,
	0b10000000,
	0b10000000,
	0b11111110,
	0b10000000,
	0b10000000,
	0b10000000,
	0b10000000,
};

const char letterG[8]={
	
	0b11111111,
	0b10000000,
	0b10000000,
	0b10111111,
	0b10000001,
	0b10000001,
	0b10000001,
	0b11111111,
};

const char letterH[8]={
	
	0b10000001,
	0b10000001,
	0b10000001,
	0b11111111,
	0b10000001,
	0b10000001,
	0b10000001,
	0b10000001,
};

const char letterI[8]={
	
	0b11111111,
	0b00011000,
	0b00011000,
	0b00011000,
	0b00011000,
	0b00011000,
	0b00011000,
	0b11111111,
};

const char letterJ[8]={
	
	0b11111111,
	0b00000001,
	0b00000001,
	0b00000001,
	0b00000001,
	0b01000001,
	0b01000001,
	0b01111111,
};

const char letterK[8]={
	
	0b10000100,
	0b10001000,
	0b10010000,
	0b10100000,
	0b11010000,
	0b10001000,
	0b10000100,
	0b10000010,
};

const char letterL[8]={
	
	0b10000000,
	0b10000000,
	0b10000000,
	0b10000000,
	0b10000000,
	0b10000000,
	0b10000000,
	0b11111111,
};

const char letterM[8]={
	
	0b10000001,
	0b11000011,
	0b10100101,
	0b10011001,
	0b10000001,
	0b10000001,
	0b10000001,
	0b10000001,
};

const char letterN[8]={
	
	0b10000001,
	0b11000001,
	0b10100001,
	0b10010001,
	0b10001001,
	0b10000101,
	0b10000011,
	0b10000001,
};

const char letterO[8]={
	
	0b01111110,
	0b10000001,
	0b10000001,
	0b10000001,
	0b10000001,
	0b10000001,
	0b10000001,
	0b01111110,
};

const char letterP[8]={
	
	0b11111110,
	0b10000001,
	0b10000001,
	0b11111110,
	0b10000000,
	0b10000000,
	0b10000000,
	0b10000000,
};

const char letterQ[8]={
	
	0b11111111,
	0b10000001,
	0b10000001,
	0b10000001,
	0b10000001,
	0b10001001,
	0b10000101,
	0b11111111,
};

const char letterR[8]={
	
	0b11111110,
	0b10000001,
	0b10000001,
	0b11111110,
	0b10001000,
	0b10000100,
	0b10000010,
	0b10000001,
};

const char letterS[8]={
	
	0b01111111,
	0b10000001,
	0b10000001,
	0b01111110,
	0b00000001,
	0b00000001,
	0b00000001,
	0b11111110,
};

const char letterT[8]={
	
	0b11111111,
	0b00010000,
	0b00010000,
	0b00010000,
	0b00010000,
	0b00010000,
	0b00010000,
	0b00010000,
};

const char letterU[8]={
	
	0b10000001,
	0b10000001,
	0b10000001,
	0b10000001,
	0b10000001,
	0b10000001,
	0b10000001,
	0b01111110,
};

const char letterV[8]={
	
	0b10000001,
	0b10000001,
	0b10000001,
	0b10000001,
	0b10000001,
	0b01000010,
	0b10100100,
	0b10011000,
};

const char letterW[8]={
	
	0b10000001,
	0b10000001,
	0b10000001,
	0b10000001,
	0b10000001,
	0b10011001,
	0b10100101,
	0b11000011,
};

const char letterX[8]={
	
	0b10000001,
	0b01000010,
	0b00100100,
	0b00011000,
	0b00011000,
	0b00100100,
	0b01000010,
	0b10000001,
		
};

const char letterY[8]={
	
	0b10000001,
	0b10000001,
	0b10000001,
	0b01111111,
	0b00000001,
	0b00000001,
	0b00000001,
	0b01111110,
};

const char letterZ[8]={
	
	0b11111111,
	0b00000010,
	0b00000100,
	0b00001000,
	0b00010000,
	0b00100000,
	0b01000000,
	0b11111111,
};

/************************************************************************/
/* @decode the value to light up segments                                                                     */
/************************************************************************/
int decode8X8matrix(char val){
	
	switch(val){
		
		case '0': return letter0;
		break;
		
		case '1': return letter1;
		break;
		
		case '2': return letter2;
		break;
		
		case '3': return letter3;
		break;
		
		case '4': return letter4;
		break;
		
		case '5': return letter5;
		break;
		
		case '6': return letter6;
		break;
		
		case '7': return letter7;
		break;
		
		case '8': return letter8;
		break;
		
		case '9': return letter9;
		break;
		
		case 'A': return letterA; // A
		break;
		
		case 'B': return letterB; // b
		break;
		
		case 'C': return letterC; // C
		break;
		
		case 'D': return letterD;// d
		break;
		
		case 'E': return letterE;// E
		break;
		
		case 'F': return letterF;// F
		break;
		
		case 'G': return letterG;// G
		break;
		
		case 'H': return letterH;// H
		break;
		
		case 'I': return letterI;// I
		break;
		
		case 'J': return letterJ;// J
		break;
		
		case 'K': return letterK;// K
		break;
		
		case 'L': return letterL;// L
		break;
		
		case 'M': return letterM;// M
		break;
		
		case 'N': return letterN;// N
		break;
		
		case 'O': return letterO;// O
		break;
		
		case 'P': return letterP;// P
		break;
		
		case 'Q': return letterQ;// Q
		break;
		
		case 'R': return letterR;// R
		break;
		
		case 'S': return letterS;// Q
		break;
		
		case 'T': return letterT;// S
		break;
		
		case 'U': return letterU;// T
		break;
		
		case 'V': return letterV;// U
		break;
		
		case 'W': return letterW;// V
		break;
		
		case 'X': return letterX;// W
		break;
		
		case 'Y': return letterY;// X
		break;
		
		case 'Z': return letterZ;// Y
		break;
		
		default: return NULL;
		break;
		
	}
	
}

/************************************************************************/
/* @clear Display                                                                     */
/************************************************************************/

void clearLedMatrix(void){
	
	uint8_t i=0;
	for(i=0; i<8; i++){
		MAX7219_sendCmd(i+1,0);
	}
}


/************************************************************************/
/* @ clear daisy chain                                                                     */
/************************************************************************/
void clearLedMatrixDaisyChain(uint8_t nMatrix){
	
	uint8_t i=0;
	for(i=0; i<8; i++){
		CS_LOW;
		for (uint8_t j=0; j<nMatrix; j++){
			MAX7219_sendCmd_NoCS(i+1,0);	
		}
		CS_HIGH;
		
	}
}


/************************************************************************/
/* @ init daisy chain                                                                     */
/************************************************************************/
void Led8x8Init_initDaisyChain(uint8_t mode, uint8_t decdMode, uint8_t dgt, uint8_t bright, uint8_t nMatrix){
	
	
uint8_t ncmd =0;	
	
	while (ncmd < 5){ // send the 4 commands consecutevely
		CS_LOW;
		for (uint8_t i =0; i< nMatrix; i++){
			//mete N_matrix no len senao nao seb
			// concatena tudo
			switch(ncmd){
				case 0: MAX7219_sendCmd_NoCS(MAX7219_DECODE_MODE,decdMode);	//set decode mode
					break;
					
				case 1: MAX7219_sendCmd_NoCS(MAX7219_SCAN_LIMIT,dgt);		//set digit number
					break;
				
				case 2: MAX7219_sendCmd_NoCS(MAX7219_BRIGHTNESS,bright);	//set brightness
					break;
				
				case 3: MAX7219_sendCmd_NoCS(MAX7219_SHUTDOWN,mode);		//set mode enable/disable
					break;
					
				case 4: MAX7219_sendCmd_NoCS(MAX7219_TEST,0);			//test display OFF;		//set mode enable/disable
				break;	
				
				default:
					break;					
			}
					
		}
		CS_HIGH;
		ncmd++;	
	}
	
	
}


/************************************************************************/
/* @ set values in dsaisy chain                                                                     */
/************************************************************************/
void Led8x8SetValueDaisyChain(char *d, uint8_t  nMatrix){
	
	for(uint8_t j=0; j <8; j++){// loop all segmentes
		// loop all caracters
		CS_LOW; // do CS
		for (uint16_t i =0; i< nMatrix; i++ ){ // for all caracters estract the seg
			char * ptrDisplay =(decode8X8matrix(d[i]));
			MAX7219_sendCmd_NoCS(j+1, (*(ptrDisplay+j))); // nao pode fazer CS
		}
		CS_HIGH;// relase CS
	}
}




/************************************************************************/
/* @init 8x8 led matrix                                                                  */
/************************************************************************/
void Led8x8Init(uint8_t mode, uint8_t decdMode, uint8_t dgt, uint8_t bright){
	
	MAX7219_init(mode,decdMode,dgt,bright);
}




/************************************************************************/
/*@set value to show                                                    */
/************************************************************************/
void Led8x8_setValue(char val){

char *ptrDisplay= decode8X8matrix(val); // pointer


	for (char i=0; i<8; i++){
		MAX7219_sendCmd( i+1, (*(ptrDisplay+i)));
	}
	
	
}


