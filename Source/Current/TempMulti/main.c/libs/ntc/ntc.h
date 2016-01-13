#ifndef NTCTEMP_H
#define NTCTEMP_H

//enable the B Model Equation
#define NTC_B
//enable the SH Model Equation
#define NTC_SH

// enable the kelvin temp mode
//#define NTC_K
// enable the celcius temp mode
#define NTC_C
// enable the farheint temp mode
//#define NTC_F





#ifdef NTC_B
extern double NTC_getBtemp(long adcResValue, int beta, double adcTref, int adcRref);
#endif

#ifdef NTC_SH
extern double NTC_getSHtemp(long adcResValues, double A, double B, double C);
#endif

#endif