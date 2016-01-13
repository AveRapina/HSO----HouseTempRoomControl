/*
 * max7219.c
 *
 * Created: 17-10-2015 17:20:24
 *  Author: Nelson
 */ 

#include "max7219.h" // estas na mesma pasta!

#include <avr/io.h>
#include "../spi/my_spi.h"
#include "../utils/bit_tools.h"


//prototypes
void MAX7219_sendCmd(uint8_t address,uint8_t command);
void MAX7219_init(uint8_t mode, uint8_t decdMode, uint8_t dgt, uint8_t bright);
uint8_t MAX7219_sendData(uint8_t address,uint8_t command);



uint8_t count=0;
uint8_t countMatrix=0;
/************************************************************************/
/* @Send Data to Max7219                                                                      */
/************************************************************************/
uint8_t MAX7219_sendData(uint8_t address,uint8_t command){
	
	
	CS_LOW;
	SPIBANG_sendChar(address);
	SPIBANG_sendChar(command);
	count++;
	if (count==8){
		countMatrix++;
		count=0;
		if (countMatrix==N_LCDMATRIX){
			CS_HIGH;
			countMatrix= 0;
		}
	}
	else return 1;
// 	CS_LOW;
// 	SPI_transceiver(address);
// 	SPI_transceiver(command);
// 	CS_HIGH;
	
}


/************************************************************************/
/* @Send Comand to Config Max7219                                                                      */
/************************************************************************/
void MAX7219_sendCmd_NoCS(uint8_t address,uint8_t command){
	
	
	//CS_LOW;
	SPIBANG_sendChar(address);
	SPIBANG_sendChar(command);
	//CS_HIGH;

	// 	CS_LOW;
	// 	SPI_transceiver(address);
	// 	SPI_transceiver(command);
	// 	CS_HIGH;
	
}



/************************************************************************/
/* @Send Comand to Config Max7219                                                                      */
/************************************************************************/
void MAX7219_sendCmd(uint8_t address,uint8_t command){
	
	
	CS_LOW;
	SPIBANG_sendChar(address);
	SPIBANG_sendChar(command);
	CS_HIGH;

	// 	CS_LOW;
	// 	SPI_transceiver(address);
	// 	SPI_transceiver(command);
	// 	CS_HIGH;
	
}




/************************************************************************/
/* @Init Max 7219                                                       */ 
/************************************************************************/
void MAX7219_init(uint8_t mode, uint8_t decdMode, uint8_t dgt, uint8_t bright){
	
	
	MAX7219_sendCmd(MAX7219_DECODE_MODE,decdMode);	//set decode mode
	MAX7219_sendCmd(MAX7219_SCAN_LIMIT,dgt);		//set digit number
	MAX7219_sendCmd(MAX7219_BRIGHTNESS,bright);	//set brightness
	MAX7219_sendCmd(MAX7219_SHUTDOWN,mode);		//set mode enable/disable	
	MAX7219_sendCmd(MAX7219_TEST,0);			//test display OFF

	
}


/*
void MAX7219_sendData(uint16_t data){
	
	CS_LOW;
	SPI_transceiver(0x01);//endereço dgt 0
	SPI_transceiver(0x02);//valor qq pra teste
	CS_HIGH;
	
}

*/
	
