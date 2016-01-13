/*
 * joy2x3.h
 *
 * Created: 08-10-2015 14:25:41
 *  Author: Nelson
 */ 
#ifndef F_CPU
#define F_CPU 16000000
#endif

#ifndef JOY2X3_H_
#define JOY2X3_H_

#include <avr/io.h>
#include <stdlib.h>
#include "../utils/bit_tools.h"
#include "../utils/my_utils.h"

#define JOY B

#define JOY_DDR DDR(JOY)
#define JOY_PORT PORT(JOY)
#define JOY_PIN	PIN(JOY)


extern char* readJoystk(void);


#endif /* JOY2X3_H_ */