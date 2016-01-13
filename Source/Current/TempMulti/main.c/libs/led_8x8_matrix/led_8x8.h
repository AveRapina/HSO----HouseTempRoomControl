/*
 * led_8x8.h
 *
 * Created: 12-10-2015 12:19:57
 *  Author: Nelson
 */ 


#ifndef LED8x8_H_
#define LED8x8_H_
#include <inttypes.h>

extern void Led8x8Init(uint8_t mode, uint8_t decdMode, uint8_t dgt, uint8_t bright);
extern void Led8x8_setValue(char val);
extern void Led8x8Init_initDaisyChain(uint8_t mode, uint8_t decdMode, uint8_t dgt, uint8_t bright, uint8_t nMatrix);
extern void Led8x8SetValueDaisyChain(char *d, uint8_t);

#endif /* LED_8x8_H_ */