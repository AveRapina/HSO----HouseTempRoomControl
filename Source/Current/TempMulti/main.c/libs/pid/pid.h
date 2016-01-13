#ifndef PID_H_
#define PID_H_

#include <inttypes.h>

#define INT_MAX 65535/2
#define INT_MIN -65354/2


//functions
extern void PID_setPid(double pidP, double pidI, double pidD) ;
extern void PID_setLimitsPerr(int16_t Perr_min, int16_t Perr_max);
extern void PID_setLimitsIerr(int16_t Ierr_min, int16_t Ierr_max);
extern void PID_resetIerr();
extern int16_t PID_update(int16_t setpoint, int16_t input, double dt);

#endif