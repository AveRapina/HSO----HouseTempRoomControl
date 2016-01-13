
// ctrl+D indent code
#include "my_utils.h"




#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include "../utils/bit_tools.h"
#include "../usart/usart.h"


// Macro NOP


// Variable for save parts
volatile char tokenParts[15][20];


/************************************************************************/
/* check index of string    -1 false                                                                  */
/************************************************************************/
int8_t indexOf(char *data, char c){
	uint8_t len=0;
	uint8_t i=0;
	len = strlen(data);
	
	for(i=0;i<len; i++){
		if(data[i]==c) return i;
	}
	return -1; // return false
}


/************************************************************************/
/*                                                                      */
/************************************************************************/
uint8_t parseString_STRTOK(char *data, char delimiter){
	char * p;
	uint8_t n = 0;

	p = strtok(data, ",");
	while (*p) {
		strcpy(&tokenParts[n++], p);
		p = strtok(NULL, ",");
	}
	return n;	
}


/************************************************************************/
/* @pasring string into tokens and put in global parts variable                                                                      */
/************************************************************************/
uint8_t parseString(char *data, char delimiterChar){
	char *p_start, *p_end;
	uint8_t nTokens=0;

	p_start = data;

	while(1) {
		p_end = strchr(p_start, delimiterChar);
		if (p_end) {
			strncpy(tokenParts[nTokens], p_start, p_end-p_start);
			tokenParts[nTokens][p_end-p_start] = 0;
			nTokens++;
			p_start = p_end + 1;
		}
		else {
			// sopy the last bit - might as well copy 20
			strncpy(tokenParts[nTokens], p_start, 20);
			break;
		}
	}
	return nTokens;
}


/************************************************************************/
/* @UsartSendDecimal                                                    */
/************************************************************************/
void sendDecUSART(uint8_t value){
	USART0_sendChar(value + '0');
	// usase a default, para isto,
	
}

/************************************************************************/
/* @UsartSendHex                                                    */
/************************************************************************/
void sendHexUSART(uint8_t value){
	uint8_t nibble_low = value;
	uint8_t nibble_high = (value>>4);
	calc_hex_nibble(nibble_low);
	calc_hex_nibble(nibble_high);

}

/************************************************************************/
/* @CalcHexNibble                                                       */
/************************************************************************/
void calc_hex_nibble(uint8_t nibble){
	uint8_t maskednibble = (nibble & 0x0f);
	if (maskednibble>9) maskednibble += ('a'-10);
	else maskednibble +='0';
	
	USART0_sendChar(maskednibble);
}

/************************************************************************/
/* @Usart Send Binary                                                   */
/************************************************************************/
void sendBinUSART(uint8_t value){
	uint8_t i;
	
	for (i=128;i>0;i/2){
		if(value&i) USART0_sendChar('1');
		else USART0_sendChar('0');
		
	}
}