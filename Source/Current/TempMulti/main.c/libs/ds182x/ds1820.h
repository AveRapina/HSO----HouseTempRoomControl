/*
 * ds1820.h
 *
 * Created: 24.08.2015 18:49:25
 *  Author: HSO
 */ 


#ifndef DS1820_H_
#define DS1820_H_

#include <avr/io.h>
#include "../utils/bit_tools.h"
#include "../utils/my_utils.h"

/************************************************************************/
/* PIN PORT                                                                     */
/************************************************************************/
#define DS18_PORT	B
#define DS18_SDA	(1<<PB0)


#define DS18_CMD_SKIPROM 0xCC
#define DS18_CMD_CONVERTTEMP 0x44
#define DS18_CMD_RSCRACTHPAD 0xBE


void DS18_reset(void);
void DS18_writeByte(uint8_t data);
uint8_t DS18_readByte(void);
extern double DS18_getTemp(void);



#endif /* DS1820_H_ */