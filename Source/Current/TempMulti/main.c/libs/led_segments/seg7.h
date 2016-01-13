/*
 * seg7.h
 *
 * Created: 08-10-2015 17:56:30
 *  Author: Nelson
 */ 

#include <avr/io.h>

#ifndef SEG7_H_
#define SEG7_H_

// #define SEG7 E
// #define SEG7_DDR DDR(SEG7)
// #define SEG7_PORT PORT(SEG7)

#define N_SEG7 4

#define ROW_ADD PCF8574_BASEADD
#define COL_ADD PCF8574_BASEADD +2

extern void SEG_init(void);
extern void SEG_setValue(uint8_t addr,int val);


// mine
extern void SEG_nextSegment(void);
extern void SEG_setNewValue(uint16_t val);





#endif /* 7SEG_H_ */