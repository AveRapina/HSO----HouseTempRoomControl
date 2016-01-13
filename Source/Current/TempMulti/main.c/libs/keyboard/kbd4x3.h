/*
 * kbd4x3.h
 *
 * Created: 06-10-2015 19:37:04
 *  Author: Nelson
 */ 


#ifndef KBD4X3_H_
#define KBD4X3_H_
#include <avr/io.h>
#include <stdlib.h>
#include "../utils/bit_tools.h"

#include "../utils/my_utils.h"



#define KEYPAD A//isso aconteceu me n de x
#define KEYPAD_PIN PIN(KEYPAD)
#define KEYPAD_PORT PORT(KEYPAD)
#define KEYPAD_DDR   DDR(KEYPAD)


extern char KBD_read(void);

#endif /* KBD4X3_H_ */