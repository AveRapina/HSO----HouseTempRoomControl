/*
 * SHT25.h
 *
 * Created: 05-08-2015 14:11:53
 * 
 */ 


#include <avr/io.h>
#include "../utils/bit_tools.h"
#include "../i2c/twi.h"

/************************************************************************/
/* SHT25 Address                                                        */
/************************************************************************/
#define SHT25_ADDRESS 0x25

/************************************************************************/
/* SHT25 Power management                                               */
/************************************************************************/
#define SHT25_POWER_ON bit_set(PORTD,5)
#define SHT25_POWER_DOWN bit_clear(PORTD,5)

/************************************************************************/
/* @SHT25 Power ON Delay                                                */
/************************************************************************/
#define SHT25_POWER_ON_DELAY 15

/************************************************************************/
/* SHT25 registers                                                      */
/************************************************************************/
#define SHT25_HEATER 0x04
#define SHT25_RH12_T14_BITS 0x00
#define SHT25_RH08_T12_BITS 0x01
#define SHT25_RH10_T13_BITS 0x80
#define SHT25_RH11_T11_BITS 0x81

/************************************************************************/
/* Types of trigger                                                     */
/************************************************************************/
#define SHT25_CMD_T_TRIGGER_HOLDMASTER 0b11100011
#define SHT25_CMD_RH_TRIGGER_HOLDMASTER 0b11100101
#define SHT25_CMD_T_TRIGGER_NOHOLDMASTER 0b11110011
#define SHT25_CMD_RH_TRIGGER_NOHOLDMASTER 0b11110101
#define SHT25_CMD_WRITE_USER_REGISTER 0b11100110
#define SHT25_CMD_READ_USER_REGISTER 0b11100111
#define SHT25_CMD_SOFT_RESET 0xfe

/************************************************************************/
/* Coefficients				                                            */
/************************************************************************/
#define BETAW 17.62
#define BETAI 22.46
#define LAMBDAW 243.12
#define LAMBDAI 272.62

/************************************************************************/
/* @prototypes                                                          */
/************************************************************************/
void SHT25_hardPowerON(uint8_t addr, uint8_t mode);
void SHT25_hardPowerOFF(void);
void SHT25_softReset(uint8_t addr);
void SHT25_init(uint8_t addr, uint8_t mode);
uint8_t SHT25_readUserReg(uint8_t addr);
uint16_t SHT25_readTemp(uint8_t addr, uint8_t cmd);
uint16_t SHT25_calcTemp(uint8_t addr, uint8_t cmd);
uint16_t SHT25_readRH(uint8_t addr, uint8_t cmd);
uint16_t SHT25_calcRH(uint8_t addr, uint8_t cmd);
uint8_t checkCRC(volatile uint8_t *data);
float SHT25_calcRHAbsolute(uint16_t temperature, uint16_t rh);


