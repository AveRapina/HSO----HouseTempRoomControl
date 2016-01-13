/*
 * seg14.h
 *
 * Created: 08-10-2015 18:36:53
 *  Author: Nelson
 */ 


#ifndef SEG14_H_
#define SEG14_H_

#include <avr/io.h>


// uncomment used driver
#define USE_PCF
//#define USE_PCA
//#define PCA_KT
#define PCA_AN


#define N_SEG14   10 // 0-9

#define COL_14_ADD PCF8575_BASEADD +4
#define ROW_14_ADD PCF8575_BASEADD +6





extern void SEG14_setNewValue(char * sentence);
extern void SEG14_nextSegment(void);


#endif /* SEG14_H_ */