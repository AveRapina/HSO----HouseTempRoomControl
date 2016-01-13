/*
 * max7219.h
 *
 * Created: 17-10-2015 17:20:44
 *  Author: Nelson
 */ 

#include <avr/io.h>
#include "../utils/my_utils.h"
#include "../utils/bit_tools.h"

extern void MAX7219_sendCmd(uint8_t address,uint8_t command);
extern void MAX7219_init(uint8_t mode, uint8_t decdMode, uint8_t dgt, uint8_t bright);
extern uint8_t MAX7219_sendData(uint8_t address,uint8_t command);
extern void MAX7219_sendCmd_NoCS(uint8_t address,uint8_t command);

#ifndef MAX7219_H_
#define MAX7219_H_


#define CS_LOW bit_clear(PORTB,0)
#define CS_HIGH bit_set(PORTB,0)

// Address Registers
#define MAX7219_TEST 0x0F
#define MAX7219_BRIGHTNESS 0x0A 
#define MAX7219_SCAN_LIMIT 0x0B 
#define MAX7219_DECODE_MODE 0x09 
#define MAX7219_SHUTDOWN 0x0C
#define MAX7219_DGT0 0x01 
#define MAX7219_DGT1 0x02 
#define MAX7219_DGT2 0x03 
#define MAX7219_DGT3 0x04 
#define MAX7219_DGT4 0x05 
#define MAX7219_DGT5 0x06 
#define MAX7219_DGT6 0x07 
#define MAX7219_DGT7 0x08 

//Commands
#define MODE_SHTDWN 0x00
#define MODE_NORMAL 0x01


#define DEC_MODE_OFF 0x00

//define the number of lcd matrixes to be used
#define N_LCDMATRIX 8

#endif /* MAX7219_H_ */