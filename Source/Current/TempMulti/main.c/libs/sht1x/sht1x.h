
#include <avr/io.h>
#include "../utils/bit_tools.h"
#include "../utils/my_utils.h"

#define SHT11_PORT	A
#define SHT11_SCL	(1<<PA0)
#define SHT11_SDA	(1<<PA1)

// define pins to be used
#define SHT11_POWER_UP 
#define SHT11_POWER_DOWN


/*
#define setBits(port,mask)	do{ (port) |=  (mask); }while(0)
#define clrBits(port,mask)	do{ (port) &= ~(mask); }while(0)
#define tstBits(port,mask)	((port) & (mask))
*/



// code erros
#define		SHT11_UNAVAIL  -32768
#define		SHT11_CRC_FAIL -32767
#define		sht11_valid(v) ((v) > -32000)

// codes config modes
#define SHT11_CMD_TEMP	0x03
#define SHT11_CMD_HUMID	0x05
#define SHT11_CMD_WSTAT	0x06
#define SHT11_CMD_RSTAT	0x07
#define SHT11_CMD_RESET	0x1E
#define SHT11_LOWRES	0

#define SHT11_CONVERT_READ 1
#define SHT11_NO_CONV_READ 0


void SHT11_hardPowerOn(void);
void SHT11_hardPowerOFF(void);


//prototypes
int8_t SHT11_startMeasure(uint8_t cmd);
int16_t SHT11_readTemp(uint8_t mode);
int16_t SHT11_readRH(uint8_t mode);
int16_t SHT11_result(void);
void  SHT11_calValues(int16_t temp, int16_t humid, float *tc, float *rhlin, float *rhtrue);


void SHT11_resetCom(void);
void SHT11_waitComFree(void);
void SHT11_transmissionStart(void);
uint8_t SHT11_ready(void);
uint8_t SHT11_writeByte(uint8_t data);
uint8_t SHT11_readByte(void);
void SHT11_init(void);

uint8_t SHT11_readCRC(void);
void SHT11_crc8(uint8_t b);


