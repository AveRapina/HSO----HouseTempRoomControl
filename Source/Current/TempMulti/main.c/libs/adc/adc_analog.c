#include "adc_analog.h"

/* define CPU frequency in Mhz here if not defined in Makefile */
#ifndef F_CPU
#define F_CPU 2000000UL
#endif



#include <avr/io.h>
#include <util/delay.h>
#include "../utils/bit_tools.h"



/************************************************************************/
/* @obtain prescaler at ADc frequancy specified                                                                     */
/************************************************************************/
uint8_t ADC_calcPreScaler(uint8_t adcClk){ // ne precisa depois alterase

	// CHECK THIS
	
	if ((F_CPU / 2) < adcClk) return ADC_PRESC_DIV2;
	else if ((F_CPU / 4) < adcClk) return ADC_PRESC_DIV4;
	else if ((F_CPU / 8) < adcClk) return ADC_PRESC_DIV8;
	else if ((F_CPU / 16) < adcClk) return ADC_PRESC_DIV16;
	else if ((F_CPU / 32) < adcClk) return ADC_PRESC_DIV32;
	else if ((F_CPU / 64) < adcClk) return ADC_PRESC_DIV64;
	else if ((F_CPU / 128) < adcClk) return ADC_PRESC_DIV128;
	else return 0;
}



/************************************************************************/
/* @init ADC                                                                    */
/************************************************************************/
void ADC_init(uint8_t mode, uint8_t voltageRef,uint8_t adcClk){
	ADCSRA |=mode;
	ADCSRA |= ADC_calcPreScaler(adcClk);
	ADMUX = voltageRef;
	
}


/*
 * convert an adc value to a resistance value
 */
long ADC_getResistence(uint16_t adcread, uint16_t adcbalanceresistor)
{
	if(adcread == 0)
		return -1;
	else
		return (long)((long)(ADC_RESOLUTION*(long)adcbalanceresistor)/adcread-(long)adcbalanceresistor);
}


/*
 * exponential moving avarage filter
 *
 * "newvalue" new adc read value
 * "value" old adc filtered value
 * return a new filtered value
 *
 * References:
 *   Guillem Planissi: Measurement and filtering of temperatures with NTC
 */
#define ADC_EMAFILTERALPHA 30
unsigned int ADC_emaFilter(unsigned int newvalue, unsigned int val)
{
	//use exponential moving avarate Y=(1-alpha)*Y + alpha*Ynew, alpha between 1 and 0
	//in uM we use int math, so Y=(63-63alpha)*Y + 63alpha*Ynew  and  Y=Y/63 (Y=Y>>6)
	val = (64-ADC_EMAFILTERALPHA)*val+ADC_EMAFILTERALPHA*newvalue;
	val = (val>>6);
	return val;
}



/************************************************************************/
/* @ CHECK how to do best                                                                     */
/************************************************************************/
uint16_t ADC_readFromINT(void){
	
	uint16_t val;
	
	val = ADCL;
	val |= (ADCH <<8);
	// clear flags for next run
	
	return val;
}



/************************************************************************/
/* @fire ADC                                                                     */
/************************************************************************/
void ADC_startConversion(uint8_t ch){
	
	ch&= 0b00000111;
	ADMUX &= 0xF8 |ch;
	
	//-- Start conv
	ADCSRA |= ADC_START_CONV;
	
	// wait until complete
	while(ADCSRA & ADC_START_CONV);
	
}



/************************************************************************/
/* @adc check if conversion finished and read                                                                     */
/************************************************************************/
uint8_t ADC_checkStatusAndRead(uint16_t *val){
	
	// wait until complete
	if(ADCSRA & ADC_START_CONV) return 0;
	
	// grab values é ler pra uma var adcl e adch
	*val = ADCL;
	*val |= (ADCH <<8);
	return 1;
	
}


/************************************************************************/
/* @read adc                                                                     */
/************************************************************************/
uint16_t ADC_readAndWAIT(uint8_t ch){
uint16_t val;
	
	
	ch &= 0b00000111;  // AND operation with 7
	ADMUX = (ADMUX & 0xF8)|ch; // clears the bottom 3 bits before ORing
	

	
	//-- Start conv
	ADCSRA |= ADC_START_CONV;
	
	// wait until complete
	while(ADCSRA & ADC_START_CONV);
	
	// grab values é ler pra uma var adcl e adch
	val = ADCL;
	val |= (ADCH <<8);
	return val;
	 
}