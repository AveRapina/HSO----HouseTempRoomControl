/*
 * ntc.c
 *
 * Created: 01.09.2015 18:01:54
 *  Author: HSO
 */ 


#include <math.h>
#include "ntc.h"




/*
* Get temperature using Beta Model Equation
*
* "adcResValue" adc readed value
* "beta" parameter
* "adcTref" 
* "adcRref
*/

#ifdef NTC_B
double NTC_getBtemp(long adcResValue, int beta, double adcTref, int adcRref){
	double t;
	// tempearure(Kelvin) =beta /(beta/tref +log(R/rref))
	t = beta / (beta/(double)adcTref + log(adcResValue/(double) adcRref));
	
	#ifdef NTC_K
	return t;
	#endif
	
	#ifdef NTC_C
	return t - 273.15;
	#endif
	
	#ifdef NTC_F
	return t * 9.0 / 5.0 + 32.0;
	#endif	
}
#endif



/*
 * get temperature using the Steinhart-Hart Thermistor Equation
 *
 * "adcResValue" adc resistence read
 * "A", "B", "C" equation parameters
 */
#ifdef NTC_SH
double NTC_getSHtemp(long adcResValue, double A, double B, double C){
	double t;
	// tempearure(Kelvin) =1 /(A + B*ln(R) + C*(ln(R)^3)
	t = log(adcResValue);
	t = 1 /(A + (B  *t) + (C * t * t * t));

	#ifdef NTC_K
	return t;
	#endif
	
	#ifdef NTC_C
	return t - 273.15;
	#endif
	
	#ifdef NTC_F
	return t * 9.0 / 5.0 + 32.0;
	#endif
}
#endif