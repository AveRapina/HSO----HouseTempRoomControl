/*
 * Usart.h
 *
 * Created: 06-08-2015 14:12:44
 *
 */ 
 
 #include <avr/io.h>

#ifndef F_CPU
#define F_CPU 16000000
#endif

/************************************************************************/
/* Define Baud                                                          */
/************************************************************************/
//COM0
#define USART0_BAUD_RATE 57600
#define USART0_MY_UBBRN ((F_CPU / (USART0_BAUD_RATE <<4))-1)// usase o valor inteiro e prontosok

// COM1
#define USART1_BAUD_RATE 57600
#define USART1_MY_UBBRN ((F_CPU / (USART1_BAUD_RATE <<4))-1)// usase o valor inteiro e prontosok


#define USART_ASSYNCRONOUS_MODE   0x00
#define USART_SYNCHRONOUS_MODE	(1<<UMSEL0)
#define USART_PARITY_EVEN			(1<<UPM01)
#define USART_PARITY_ODD			(1<<UPM01)|(1<<UPM00)
#define USART_PARITY_NONE			0x00
#define USART_INTERRUPT_ENABLE (1<<RXCIE0)
#define USART_TRANSMIT_ENABLE (1<<TXEN0)
#define USART_RECEIVE_ENABLE (1<<RXEN0)
#define USART_DATA_FORMAT_9BITS (1<<UCSZ02) |(1<<UCSZ01)|(1<<UCSZ00)
#define USART_DATA_FORMAT_8BITS (1<<UCSZ01)|(1<<UCSZ00)
#define USART_DATA_FORMAT_7BITS (1<<UCSZ01)
#define USART_DATA_FORMAT_6BITS (1<<UCSZ00)
#define USART_DATA_FORMAT_5BITS 0x00
#define USART_STOP_BITS_1	0x00
#define USART_STOP_BITS_2	(1<<USBS0) 








#define MSG_END_CARACTER '|'
#define USART_BUFFER_LENGHT 50

/************************************************************************/
/* @Prototypes                                                          */
/************************************************************************/

void USART0_config(uint16_t MY_UBBR, uint8_t dataFormat, uint8_t mode);
void USART0_sendChar(char data);
void USART0_sendStr(char *data);
char USART0_receiveChar(void);

void USART1_config(uint16_t MY_UBBR, uint8_t dataFormat, uint8_t mode);
void USART1_sendChar(char data);
void USART1_sendStr(char *data);
char USART1_receiveChar(void);
