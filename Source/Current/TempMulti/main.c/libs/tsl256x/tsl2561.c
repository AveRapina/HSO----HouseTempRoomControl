

#include "tsl2561.h"
#ifndef F_CPU
#define F_CPU 2000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include "../utils/bit_tools.h"


/************************************************************************/
/* @softPowerOffTSL(void)                                                                     */
/************************************************************************/
void TSL_softPowerOff(uint8_t addr){
	
	I2C_start();
	I2C_sendAddr(addr); // Modo escrita
	I2C_sendByte(0x80); // Program timing register
	I2C_sendByte(0x00); // Power off device
	I2C_stop();
}


/************************************************************************/
/* @PowerUp TSL                                                                     */
/************************************************************************/
void TSL_softPowerOn(uint8_t addr){
	I2C_start();
	I2C_sendAddr(addr); // Modo escrita
	I2C_sendByte(0x80); // Program timing register
	I2C_sendByte(0x03); // Power up device
	I2C_stop();	
}


/************************************************************************/
/* @hardPowerOnTSL                                             */
/************************************************************************/
void TSL_hardPowerOn(uint8_t addr,uint8_t tInt){
	TSL_POWER_UP;
	_delay_ms(TSL_POWER_ON_DELAY);// expects integer
	TSL_init_NOINT(addr,tInt);
	TSL_softPowerOn(addr);
	
}


/************************************************************************/
/* @hardPowerOffTSL                                                                     */
/************************************************************************/
void TSL_hardPowerOff(void){
	TSL_POWER_DOWN;
}


/************************************************************************/
/* @SetUpTimingRegisters                                                  */
/************************************************************************/
void TSL_setupTiming(uint8_t addr, uint8_t tInt){
	
	I2C_start();
	I2C_sendAddr(addr);	//slave address
	I2C_sendAddr(0x81); //command to write on 0x81 register
	I2C_sendByte(tInt); //data to write
	I2C_stop();
	
}
/************************************************************************/
/* @initTSL_INT                                                             */
/********************************n****************************************/
void TSL_init_INT(uint8_t addr, uint8_t tInt, uint16_t lowTh, uint16_t highTh, uint8_t nCycles){
	
	I2C_start();
	I2C_sendAddr(addr);	//slave address
	I2C_sendAddr(0x81); //command to write on 0x81 register
	I2C_sendByte(tInt); //data to write
		
		
	I2C_repeatStart();
	I2C_sendAddr(addr); // Modo escrita
	I2C_sendByte(0x82); // Lower Th
	I2C_sendByte((uint8_t)(lowTh&0x00ff)); // Power up device
	
	I2C_repeatStart();
	I2C_sendAddr(addr); // Modo escrita
	I2C_sendByte(0x83); // Lower Th
	I2C_sendByte((uint8_t)((lowTh>>8) &0x00ff)); // Power up device

	I2C_repeatStart();
	I2C_sendAddr(addr); // Modo escrita
	I2C_sendByte(0x84); // Lower Th
	I2C_sendByte((uint8_t)(highTh&0x00ff)); // Power up device
	
	I2C_repeatStart();
	I2C_sendAddr(addr); // Modo escrita
	I2C_sendByte(0x85); // Lower Th
	I2C_sendByte((uint8_t)((highTh>>8) &0x00ff)); // Power up device

	I2C_sendAddr(addr); // Modo escrita
	I2C_sendByte(0x86); // Program Interrupt Register
	I2C_sendByte(nCycles); // Any value ou th Interruption
	I2C_stop();
}


/************************************************************************/
/* @initTSL                                                             */
/************************************************************************/
void TSL_init_NOINT(uint8_t addr, uint8_t tInt){
		
	TSL_setupTiming(addr,tInt);
	
	I2C_start();
	I2C_sendAddr(addr); // Modo escrita
	I2C_sendByte(0x86); // Program Interrupt Register
	I2C_sendByte(0x00); // Disable Interruption
	I2C_stop();
	
}

/************************************************************************/
/* @readTSLInfrared()                                                                     */
/************************************************************************/
uint16_t TSL_readCH1(uint8_t addr){
	
	uint16_t data;
	
	//PowerUp TSL
	I2C_start();
	I2C_sendAddr(addr); // Modo escrita
	I2C_sendByte(0x80); // Program control register
	I2C_sendByte(0x03); // Power up
	I2C_stop();
	
	I2C_start();							// READ BLOCK 
	I2C_sendAddr(addr);						//	
	I2C_sendByte(0x8E); // Command LSB		// 1st READ LSB
											//	 
	I2C_repeatStart();						//	
	I2C_sendAddr(addr+1); // reading mode	//		
	data = I2C_receiveByte_NACK();			//
	I2C_stop();								//
											//	THEN
	I2C_start();							//	READ MSB
	I2C_sendAddr(addr);						//
	I2C_sendByte(0x8F); // Command MSB		//	
											//			
	I2C_repeatStart();						//	
	I2C_sendAddr(addr+1);					//
	data |= (I2C_receiveByte_NACK() <<8);	//
	I2C_stop();								//
	
	return data;
}

/************************************************************************/
/* @readTSLVisible()                                                                     */
/************************************************************************/
uint16_t TSL_readCH0(uint8_t addr){
	
	uint16_t data;
	
	I2C_start();
	I2C_sendAddr(addr);
	I2C_sendByte(0x8C); // Command LSB
	
	I2C_repeatStart();
	I2C_sendAddr(addr+1); // reading mode
	data = I2C_receiveByte_NACK();
	I2C_stop();
	
	I2C_start();
	I2C_sendAddr(addr);
	I2C_sendByte(0x8D); // Command MSB
	
	I2C_repeatStart();
	I2C_sendAddr(addr+1);
	data |= (I2C_receiveByte_NACK() <<8);
	I2C_stop();
	
	return data; 
}


/************************************************************************/
/* @CalculateLux                                                        */
/************************************************************************/
uint16_t TSL_calculateLux(uint16_t iGain, uint16_t tInt, uint16_t ch0, uint16_t ch1, uint8_t iType){

/************************************************************************/
/* The scale channel depends on the gain  integration time				*/
/* 16X, 402ms nominal                                                   */
/************************************************************************/
unsigned long chScale;
unsigned long channel0;
unsigned long channel1;
unsigned long lux;
unsigned long ratio;
unsigned long ratio1;
unsigned long b=0;
unsigned long m=0;
unsigned long temp;
	
	switch(tInt){
		case 0: chScale =TSL_CH_SCALE_TINT0; // 13.7msec
				break;
				
		case 1: chScale = TSL_CH_SCALE_TINT1; // 101msec
				break;
				
		default:chScale = (1<<TSL_CH_SCALE); // NOSCALE  //
				break;
	}		
		// scale if gain is NOT 16X
		if (!iGain) chScale = chScale << 4; // scale 1X to 16X
		// scale the channel values
		channel0 = (ch0 * chScale) >> TSL_CH_SCALE;
		channel1 = (ch1 * chScale) >> TSL_CH_SCALE;
		//????????????????????????????????????????????????????????????????????????
		// find the ratio of the channel values (Channel1/Channel0)
		// protect against divide by zero
		ratio1 = 0;
		if (channel0 != 0) ratio1 = (channel1 << (TSL_RATIO_SCALE+1)) / channel0;
		// round the ratio value
		ratio = (ratio1 + 1) >> 1;
		// is ratio <= eachBreak ?
		
		switch (iType) {
			
				case 0: // T, FN and CL package
						if ((ratio >= 0) && (ratio <= K1T)){b=B1T; m=M1T;}
						else if (ratio <= K2T){b=B2T; m=M2T;}
						else if (ratio <= K3T){b=B3T; m=M3T;}
						else if (ratio <= K4T){b=B4T; m=M4T;}
						else if (ratio <= K5T){b=B5T; m=M5T;}
						else if (ratio <= K6T){b=B6T; m=M6T;}
						else if (ratio <= K7T){b=B7T; m=M7T;}
						else if (ratio > K8T){b=B8T; m=M8T;}
						break;
						
				case 1:// CS package
						if ((ratio >= 0) && (ratio <= K1C)){b=B1C; m=M1C;}
						else if (ratio <= K2C){b=B2C; m=M2C;}
						else if (ratio <= K3C){b=B3C; m=M3C;}
						else if (ratio <= K4C){b=B4C; m=M4C;}
						else if (ratio <= K5C){b=B5C; m=M5C;}
						else if (ratio <= K6C){b=B6C; m=M6C;}
						else if (ratio <= K7C){b=B7C; m=M7C;}
						else if (ratio > K8C){b=B8C; m=M8C;}
						break;	
				}
				
		temp = ((channel0 * b) - (channel1 * m));
		// do not allow negative lux value
		if (temp < 0) temp = 0;
		// round lsb (2^(LUX_SCALE-1))
		temp += (1 << (TSL_LUX_SCALE-1));
		// strip off fractional portion
		lux = temp >> TSL_LUX_SCALE;
				
			
	return(lux);	
}