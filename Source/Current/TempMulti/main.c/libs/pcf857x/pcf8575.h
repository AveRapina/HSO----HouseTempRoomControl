/*
 * pcf8575.h
 *
 * Created: 08-10-2015 18:33:28
 *  Author: Nelson
 */ 


#ifndef PCF8575_H_
#define PCF8575_H_

#include <inttypes.h>

#define PCF8575_BASEADD 0x40
#define PCF8575A_BASEADD 0x70
#define N_PCF_DEVICES 1

//extern volatile uint16_t pcfPinVal;

extern uint16_t PCF16_writePin(uint8_t addr, uint16_t pin, uint16_t val);
extern int16_t PCF16_readPin(uint8_t addr, uint16_t pin);
extern uint16_t PCF16_read(uint8_t addr);
extern void PCF16_write(uint8_t addr, uint16_t data);



#endif /* PCF8575_H_ */