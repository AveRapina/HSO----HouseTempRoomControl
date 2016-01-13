/*
 * SHT25.c
 *
 * Created: 05-08-2015 14:18:21
 *
 */ 



#include "sht2x.h"

/* define CPU frequency in Mhz here if not defined in Makefile */
#ifndef F_CPU
#define F_CPU 2000000UL
#endif

#include <util/delay.h>
#include <math.h>

/************************************************************************/
/* @hardpowerSHT                                                        */
/************************************************************************/
void SHT25_hardPowerON(uint8_t addr, uint8_t mode){

	SHT25_POWER_ON;															// AFTER POWER-UP THIS SENSOR NEEDS 15 ms WHILE SCL IS HIGH FOR
	_delay_ms (SHT25_POWER_ON_DELAY);										// REACHING IDLE STATE AND TO BE READY TO ACCEPT COMMANDS
	
	SHT25_init(addr,mode);
	
}

/****************************d********************************************/
/* @hardpowerDown                                                                     */
/************************************************************************/
void SHT25_hardPowerOFF(void){
	SHT25_POWER_DOWN;
}

/************************************************************************/
/* @softreset SHT25                                                     */
/************************************************************************/
void SHT25_softReset(uint8_t addr){
	I2C_start();
	I2C_sendAddr(addr);
	I2C_sendByte(SHT25_CMD_SOFT_RESET);
	I2C_stop();
}

/************************************************************************/
/* @initSHT25                                                           */
/************************************************************************/
void SHT25_init(uint8_t addr, uint8_t mode){			//??MODE?? HOLD/NO-HOLD? MAS ISSO É O COMANDO!?!?
//aqui e preciso mais uma coisas

	I2C_start();
	I2C_sendAddr(addr);
	I2C_sendByte(SHT25_CMD_WRITE_USER_REGISTER);
	I2C_sendByte(mode);
	I2C_stop();
}

/************************************************************************/
/* @read user register                                                  */
/************************************************************************/
uint8_t SHT25_readUserReg(uint8_t addr){
uint8_t data;
	
	I2C_start();
	I2C_sendAddr(addr);
	I2C_sendByte(SHT25_CMD_READ_USER_REGISTER);
	I2C_repeatStart();
	I2C_sendAddr(addr+1);
	data = I2C_receiveByte_NACK();	
	I2C_stop();
	
	return data;
}


/************************************************************************/
/* readTempSHT25                                                        */
/************************************************************************/
uint16_t SHT25_readTemp(uint8_t addr, uint8_t cmd){
	uint16_t data;
		
		I2C_start();
		I2C_sendAddr(addr); 
		I2C_sendByte(cmd);
		I2C_stop();
		 
		I2C_start();
		I2C_sendAddr(addr+1);				
		// aqui podemos fazer polling a scl
		_delay_us(25);						
		
		data = (uint16_t) (I2C_receiveByte_ACK()<<8);	//RECEIVE MSB FIRST
		data |= I2C_receiveByte_NACK();					//THEN RECEIVE LSB
		I2C_stop();
		
		
		return data;
	//CHECK
}

/************************************************************************/
/* @calcTempSHT25                                                       */
/************************************************************************/
uint16_t SHT25_calcTemp(uint8_t addr, uint8_t cmd){
	uint16_t sigTemp;
	float tempValue = 0;								
	
		sigTemp = SHT25_readTemp(addr,cmd);
		sigTemp &= 0xFFFC; //Last two bits are status bits and must be set to 0 for calculation of physical values
		tempValue = -46.85 + (175.72 * ((float)(sigTemp)/(float)(2^16)));	
		
		//to finish//
		return tempValue;
}

/************************************************************************/
/* readRHSHT25                                                          */
/************************************************************************/
uint16_t SHT25_readRH(uint8_t addr, uint8_t cmd){
	uint16_t data;
		
		I2C_start();
		I2C_sendAddr(addr);
		I2C_sendByte(cmd);
		I2C_stop();
		// pode ser repeated								
		I2C_start();
		I2C_sendAddr(addr+1);
		_delay_us(25);				
		
		data = (uint16_t) (I2C_receiveByte_ACK()<<8);	//RECEIVE MSB FIRST
		data = (uint8_t) (I2C_receiveByte_NACK());		//THEN RECEIVE LSB
		I2C_stop();
		return data;								
}

/************************************************************************/
/* @Calc Absolute Value Relative Humidity                               */
/************************************************************************/
float SHT25_calcRHAbsolute(uint16_t temperature, uint16_t rh){

float rhAbs;
	
	rhAbs = rh * exp ((BETAW*temperature)/(LAMBDAW+temperature)) / exp ((BETAI*temperature)/(LAMBDAI+temperature));
	return rhAbs;
}

/************************************************************************/
/* @checkcrc                                                            */
/************************************************************************/
uint8_t checkCRC(volatile uint8_t *data){
uint8_t i;
uint8_t crc=0;
uint8_t b;

	for(i = 0; i< 2; i++){
		crc ^= data[i];
		for(b=8; b >0; b--){
			if(crc & 0x80) crc = (crc << 1) & 0x131;
			else crc = (crc << 1);
		}
	}
	if(crc != data[2]) return 0;
	else return 1;
	
}


/************************************************************************/
/* @calcRHSHT25                                                         */
/************************************************************************/
uint16_t SHT25_calcRH(uint8_t addr, uint8_t cmd){
	uint16_t sigRH;
	float rhValue = 0;									
	
	sigRH = SHT25_readRH(addr,cmd);
	sigRH &= 0xFFFC; //Last two bits are status bits and must be set to 0 for calculation of physical values
	rhValue = -6 + (125 * ((float)(sigRH)/((float)(2^16))));	
	return rhValue;
}