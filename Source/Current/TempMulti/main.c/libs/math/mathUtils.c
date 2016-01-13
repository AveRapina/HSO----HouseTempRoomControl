 
#include <inttypes.h>
#include <math.h>
 
 #include "mathUtils.h"
 
 double findPeak(double *x, unsigned int len){
 	
 	uint16_t kp;
 	uint16_t maxPeak;
 	
	 if(len >0){
 		maxPeak = x[0];
 		for(uint16_t i =1; i<len-1; i++){
 			if (x[i] >maxPeak){
 				maxPeak=x[i];
 				kp=i;
 			}
 		}
 		double kpEst = kp + 0.5 * (abs(x[kp-1] - x[kp+1] ))/(abs(x[kp-1]) -2*abs(x[kp]) +abs(x[kp]));
 		double apEst =abs(kpEst) - 0.25*(abs(x[kp-1]) - abs(x[kp+1]))*(kpEst - kp);//tb façta aqui um
 		return apEst;
 	}
 	return -1;
 }
 	