#include <avr/io.h>
#include "../spi/my_spi.h"
#include "../utils/bit_tools.h"


//=============================================================================
#define ADE_CS_PORT	A
#define ADE_CS_PIN	(1<<PA2)
//=============================================================================


//=============================================================================
//Registers
#define WAVEFORM 	0x01
#define AENERGY 	0x02
#define RAENERGY	0x03
#define LAENERGY 	0x04
#define VAENERGY 	0x05
#define RVAENERGY 	0x06
#define LVAENERGY 	0x07
#define LVARENERGY 	0x08
#define MODE 		0x09
#define IRQEN 		0x0A
#define STATUS		0x0B
#define RSTSTATUS 	0x0C
#define CH1OS 		0x0D
#define CH2OS 		0x0E
#define GAIN 		0x0F
#define PHCAL 		0x10
#define APOS 		0x11
#define WGAIN 		0x12
#define WDIV 		0x13
#define CFNUM 		0x14
#define CFDEN 		0x15
#define IRMS 		0x16
#define VRMS 		0x17
#define IRMSOS 		0x18
#define VRMSOS 		0x19
#define VAGAIN 		0x1A
#define VADIV 		0x1B
#define LINECYC 	0x1C
#define ZXTOUT 		0x1D
#define SAGCYC 		0x1E
#define SAGLVL 		0x1F
#define IPKLVL 		0x20
#define VPKLVL 		0x21
#define IPEAK 		0x22
#define RSTIPEAK 	0x23
#define VPEAK 		0x24
#define RSTVPEAK	0x25
#define TEMP 		0x26
#define PERIOD 		0x27
#define TMODE 		0x3D
#define CHKSUM 		0x3E
#define DIEREV 		0X3F
//=============================================================================
// bits
// MODE register
#define DISHPF   0x0001
#define DISLPF2  0x0002
#define DISCF    0x0004
#define DISSAG   0x0008
#define ASUSPEND 0x0010
#define TEMPSEL  0x0020
#define SWRST    0x0040
#define CYCMODE  0x0080
#define DISCH1   0x0100
#define DISCH2   0x0200
#define SWAP     0x0400
#define DTRT1    0x0800
#define DTRT0    0x1000
#define WAVSEL1  0x2000
#define WAVSEL0  0x4000
#define POAM     0x8000
//=============================================================================
// Constants
#define GAIN_1    0x0
#define GAIN_2    0x1
#define GAIN_4    0x2
#define GAIN_8    0x3
#define GAIN_16   0x4
#define INTEGRATOR_ON 1
#define INTEGRATOR_OFF 0
#define CLKIN 3579545
#define NUMCYC  20
#define kU		0.000200919
#define kI		0.000020009
#define kE		0.00015683
#define kT		447443.125
//=============================================================================
// Status
#define AEHF      0x0001
#define SAG       0x0002
#define CYCEND    0x0004
#define WSMP      0x0008
#define ZX        0x0010
#define TEMPREADY 0x0020
#define RESET     0x0040
#define AEOF      0x0080
#define PKV       0x0100
#define PKI       0x0200
#define VAEHF     0x0400
#define VAEOF     0x0800
#define ZXTO      0x1000
#define PPOS      0x2000
#define PNEG      0x4000
#define RESERVED  0x8000

#define WRITE 0x80
#define READ	0
//=============================================================================



uint8_t ADE7753_read8(uint8_t addr);
uint16_t ADE7753_read16(uint8_t addr);
uint32_t ADE7753_read24(uint8_t addr);
void ADE7753_write8(uint8_t addr,uint8_t wData);
void ADE7753_write16(uint8_t addr,uint16_t wData);
uint32_t ADE_getVRMS(void);
uint32_t ADE_getIRMS(void);
uint32_t ADE_getVA(void);
uint32_t ADE_getWATT(void);
uint32_t ADE_getPERIOD(void);

