
#include <avr/io.h>
#include "../utils/my_utils.h"
#include "../utils/bit_tools.h"
#include "../i2c/twi.h"


#define PCF8574_BASEADD 0x40
#define PCF8574A_BASEADD 0x70
#define N_PCF_DEVICES 1

extern volatile uint8_t pcfPinVal;

extern int8_t PCF_writePin(uint8_t addr, uint8_t pin, uint8_t val);
extern int8_t PCF_readPin(uint8_t addr, uint8_t pin);
extern uint8_t PCF_read(uint8_t addr);
extern void PCF_write(uint8_t addr, uint8_t data);