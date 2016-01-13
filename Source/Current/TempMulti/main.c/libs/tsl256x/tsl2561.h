#include <avr/io.h>
#include "../i2c/twi.h"
#include "../utils/bit_tools.h"

/************************************************************************/
/* Define I/O                                                           */
/************************************************************************/
#define TSL_POWER_UP bit_set(PORTD,4)
#define TSL_POWER_DOWN bit_clear(PORTD,4)
#define TSL_POWER_ON_DELAY 4
/************************************************************************/
/* TSL address                                                          */  
/************************************************************************/
#define TSL_ADDR 0x29
#define TSL_CMD_CH1_READ_MSB 0x8F
#define TSL_CMD_CH1_READ_LSB 0x8E

#define TSL_CMD_CH0_READ_MSB 0x8D
#define TSL_CMD_CH0_READ_LSB 0x8C

#define TSL_CMD_POWER_UP 0x03
#define TSL_CMD_DISABLE_INT 0x00

#define TSL_REG_CONTROL 0x80
#define TSL_REG_INT_CONTROL 0x86
#define TSL_REG_TINT 0x81
/************************************************************************/
/* TSL Integration times                                                */
/************************************************************************/
#define TSL_TINT_013MS 0x00
#define TSL_TINT_101MS 0x01
#define TSL_TINT_402MS 0x02
/************************************************************************/
/* TSL SCALE                                                            */
/************************************************************************/
#define TSL_LUX_SCALE 14 // scale 2^14
#define TSL_RATIO_SCALE 9 // scale 2^9 
/************************************************************************/
/* Scale Integration                                                    */
/************************************************************************/
#define TSL_CH_SCALE_TINT0 0x7517 // 322/81 * 2^TSL_CHSCALE
#define TSL_CH_SCALE_TINT1 0x0fe7 // 322/11 * 2^TSL_CHSCALE
#define TSL_CH_SCALE 10 // scale channel values bt 2^10
/************************************************************************/
/* T package coefficients                                               */
/************************************************************************/
#define K1T 0x0040 // 0.125 * 2^RATIO_SCALE
#define B1T 0x01f2 // 0.0304 * 2^LUX_SCALE
#define M1T 0x01be // 0.0272 * 2^LUX_SCALE
#define K2T 0x0080 // 0.250 * 2^RATIO_SCALE 
#define B2T 0x0214 // 0.0325 * 2^LUX_SCALE
#define M2T 0x02d1 // 0.0440 * 2^LUX_SCALE
#define K3T 0x00c0 // 0.375 * 2^RATIO_SCALE
#define B3T 0x023f // 0.0351 * 2^LUX_SCALE
#define M3T 0x037b // 0.0544 * 2^LUX_SCALE
#define K4T 0x0100 // 0.50 * 2^RATIO_SCALE
#define B4T 0x0270 // 0.0381 * 2^LUX_SCALE
#define M4T 0x03fe // 0.0624 * 2^LUX_SCALE
#define K5T 0x0138 // 0.61 * 2^RATIO_SCALE
#define B5T 0x016f // 0.0224 * 2^LUX_SCALE
#define M5T 0x01fc // 0.0310 * 2^LUX_SCALE
#define K6T 0x019a // 0.80 * 2^RATIO_SCALE
#define B6T 0x00d2 // 0.0128 * 2^LUX_SCALE
#define M6T 0x00fb // 0.0153 * 2^LUX_SCALE
#define K7T 0x029a // 1.3 * 2^RATIO_SCALE
#define B7T 0x0018 // 0.00146 * 2^LUX_SCALE
#define M7T 0x0012 // 0.00112 * 2^LUX_SCALE
#define K8T 0x029a // 1.3 * 2^RATIO_SCALE
#define B8T 0x0000 // 0.000 * 2^LUX_SCALE
#define M8T 0x0000 // 0.000 * 2^LUX_SCALE

/************************************************************************/
/* CS package coefficients                                              */
/************************************************************************/
#define K1C 0x0043 // 0.130 * 2^RATIO_SCALE
#define B1C 0x0204 // 0.0315 * 2^LUX_SCALE
#define M1C 0x01ad // 0.0262 * 2^LUX_SCALE
#define K2C 0x0085 // 0.260 * 2^RATIO_SCALE
#define B2C 0x0228 // 0.0337 * 2^LUX_SCALE
#define M2C 0x02c1 // 0.0430 * 2^LUX_SCALE
#define K3C 0x00c8 // 0.390 * 2^RATIO_SCALE
#define B3C 0x0253 // 0.0363 * 2^LUX_SCALE
#define M3C 0x0363 // 0.0529 * 2^LUX_SCALE
#define K4C 0x010a // 0.520 * 2^RATIO_SCALE
#define B4C 0x0282 // 0.0392 * 2^LUX_SCALE
#define M4C 0x03df // 0.0605 * 2^LUX_SCALE
#define K5C 0x014d // 0.65 * 2^RATIO_SCALE
#define B5C 0x0177 // 0.0229 * 2^LUX_SCALE
#define M5C 0x01dd // 0.0291 * 2^LUX_SCALE
#define K6C 0x019a // 0.80 * 2^RATIO_SCALE
#define B6C 0x0101 // 0.0157 * 2^LUX_SCALE
#define M6C 0x0127 // 0.0180 * 2^LUX_SCALE
#define K7C 0x029a // 1.3 * 2^RATIO_SCALE
#define B7C 0x0037 // 0.00338 * 2^LUX_SCALE
#define M7C 0x002b // 0.00260 * 2^LUX_SCALE
#define K8C 0x029a // 1.3 * 2^RATIO_SCALE
#define B8C 0x0000 // 0.000 * 2^LUX_SCALE
#define M8C 0x0000 // 0.000 * 2^LUX_SCALE

// Prototypes
void TSL_softPowerOff(uint8_t addr);
void TSL_softPowerOn(uint8_t addr);
void TSL_hardPowerOn(uint8_t addr,uint8_t tInt);
void TSL_hardPowerOff(void);
void TSL_init_NOINT(uint8_t addr, uint8_t tInt);
void TSL_init_INT(uint8_t addr, uint8_t tInt, uint16_t lowTh, uint16_t highTh, uint8_t N);
uint16_t TSL_readCH1(uint8_t addr);
uint16_t TSL_readCH0(uint8_t addr);
void TSL_setupTiming(uint8_t addr, uint8_t tInt);
uint16_t TSL_calculateLux(uint16_t iGain, uint16_t tInt, uint16_t ch0, uint16_t ch1, uint8_t iType);

