#include <inttypes.h>
#include <avr/io.h>


#define ADC_ENABLE (1<<ADEN)
#define ADC_ENABLE_INT (1<<ADIE)
#define ADC_START_CONV (1<<ADSC)
#define ADC_PRESC_DIV2 (1<<ADPS0)
#define ADC_PRESC_DIV4 (1<<ADPS1)
#define ADC_PRESC_DIV8 (1<<ADPS0) | (1<<ADPS1)
#define ADC_PRESC_DIV16 (1<<ADPS2)
#define ADC_PRESC_DIV32 (1<<ADPS2) | (1<<ADPS0)
#define ADC_PRESC_DIV64 (1<<ADPS2) | (1<<ADPS1)
#define ADC_PRESC_DIV128 (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0)
#define ADC_LEFT_JUSTIFIED (1<<ADLAR)
#define ADC_RIGHT_JUSTIFIED 0x00
#define ADC_REF_INTERNAL_256 (1<<REFS0) |(1<<REFS1)  
#define ADC_REF_VCC (1<<REFS0)
#define ADC_REF_EXTERNAL 0x00
#define ADC_MAX_FREQ 125000 //Khz
#define ADC_RESOLUTION 1023
#define ADC_REF_VALUE 5
// Macro functions

#define ADC_getVolts(adcread,adcvref) (double)((double)adcread*adcvref/ADC_RESOLUTION)


// prototipes
extern uint8_t ADC_calcPreScaler(uint8_t adcClk);
extern void ADC_init(uint8_t mode, uint8_t voltageRef,uint8_t adcClk);
extern uint16_t ADC_readFromINT(void);
extern void ADC_startConversion(uint8_t ch);
extern uint8_t ADC_checkStatusAndRead(uint16_t *val);
extern uint16_t ADC_readAndWAIT(uint8_t ch);

extern long ADC_getResistence(uint16_t adcread, uint16_t adcbalanceresistor);
extern unsigned int ADC_emaFilter(unsigned int newvalue, unsigned int val);

