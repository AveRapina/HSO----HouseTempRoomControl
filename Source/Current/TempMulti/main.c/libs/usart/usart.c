


#include "usart.h"

#ifndef F_CPU
#define F_CPU 16000000
#endif

#include <avr/io.h>
#include <string.h>
#include "../utils/bit_tools.h"



///////////////////////// COM1 ///////////////////////////////////////////
/************************************************************************/
/*                                                                      */
/************************************************************************/
void USART1_config(uint16_t MY_UBBR, uint8_t dataFormat, uint8_t mode){
	
	UBRR1H	=	(unsigned char )(MY_UBBR>>8);					//LOAD UPPER-8 BITS OF UBRR VALUE in register High
	UBRR1L	=	(unsigned char )MY_UBBR;						//LOAD LOWER-8 BITS OF UBRR VALUE IN REGISTER LOW
	
	UCSR1B	=	mode;//|(1<<UDRIE0);//ENABLE TX, RX AND DATA REGISTER EMPTY ENABLE INTERRUPT USART
	//REGISTER 0 TO TRANSMIT AND RECEIVE DATA
	//ENABLE USART RECEIVE COMPLETE INTERRUPT
	
	UCSR1C  =  dataFormat;//atnçao que vai usar os mesmos paramentros ka outra
}

/************************************************************************/
/* @USART SEND CHAR                                                     */
/************************************************************************/
void USART1_sendChar(char data){
	
	while ( ( UCSR1A & (1<<UDRE1))==0 );		// DO NOTHING UNTIL UDR BECOME READY TO RECEIVE DATA
	
	UDR1	=	data;							//RECEIVING DATA FROM THE VARIABLE TO UDR0 AND SEND VIA USART TX
}

/************************************************************************/
/* @USART SEND STRING                                                   */
/************************************************************************/
void USART1_sendStr(char *data){
	int len, i;

	len = strlen(data);			//STORE THE ARRAY SIZE IN VARIABLE (LEN)
	for (i=0; i<len; i++)
	{						// FOR CYCLE FROM BEGINNING OF CHAR ARRAY TILL THE END (LEN VALUE)
		if(data[i]=='\0')		//CHECK EMPTY DATA INSIDE CHAR ARRAY
		{
			return;
		}
		else
		{
			USART1_sendChar(*(data+i));	//CALL FUNCTION AND SEND CHAR IN THAT POSITION
		}
	}

}

/************************************************************************/
/* USART RECEIVE CHAR                                                   */
/************************************************************************/
char USART1_receiveChar(void){
	//wait for data to be received
	while ( !(UCSR1A & (1<<RXC1)) );	// do nothing
	
	return UDR1;
}

///////////////////////// COM0 ///////////////////////////////////////////
/************************************************************************/
/* @USART SETUP                                                         */
/************************************************************************/
void USART0_config(uint16_t MY_UBBR, uint8_t dataFormat, uint8_t mode){
	
	UBRR0H	=	(unsigned char )(MY_UBBR>>8);					//LOAD UPPER-8 BITS OF UBRR VALUE in register High
	UBRR0L	=	(unsigned char )MY_UBBR;						//LOAD LOWER-8 BITS OF UBRR VALUE IN REGISTER LOW
	
	UCSR0B	=	mode;//|(1<<UDRIE0);//ENABLE TX, RX AND DATA REGISTER EMPTY ENABLE INTERRUPT USART
	//REGISTER 0 TO TRANSMIT AND RECEIVE DATA
	//ENABLE USART RECEIVE COMPLETE INTERRUPT
	
	UCSR0C  =  dataFormat;
}

/************************************************************************/
/* @USART SEND CHAR                                                     */
/************************************************************************/
void USART0_sendChar(char data){
	
	while ( ( UCSR0A & (1<<UDRE0))==0 );		// DO NOTHING UNTIL UDR BECOME READY TO RECEIVE DATA
	
	UDR0	=	data;							//RECEIVING DATA FROM THE VARIABLE TO UDR0 AND SEND VIA USART TX
}

/************************************************************************/
/* @USART SEND STRING                                                   */
/************************************************************************/
void USART0_sendStr(char *data){
	int len, i;

	len = strlen(data);			//STORE THE ARRAY SIZE IN VARIABLE (LEN)
	for (i=0; i<len; i++)
	{						// FOR CYCLE FROM BEGINNING OF CHAR ARRAY TILL THE END (LEN VALUE)
		if(data[i]=='\0')		//CHECK EMPTY DATA INSIDE CHAR ARRAY
		{
			return;
		}
		else
		{
			USART0_sendChar(*(data+i));	//CALL FUNCTION AND SEND CHAR IN THAT POSITION
		}
	}

}

/************************************************************************/
/* USART RECEIVE CHAR                                                   */
/************************************************************************/
char USART0_receiveChar(void){
	//wait for data to be received
	while ( !(UCSR0A & (1<<RXC0)) );	// do nothing
	
	return UDR0;
}

