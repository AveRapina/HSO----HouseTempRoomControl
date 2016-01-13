#include <avr/interrupt.h>
#include "../usart/usart.h"


// prototypes to use
ISR(USART1_RX_vect);
ISR(USART0_RX_vect);
ISR(TIMER0_OVF_vect);
ISR(TIMER1_OVF_vect);
ISR(TIMER2_OVF_vect);


// e preciso colocar todas, senao nao inclui depois se usarmos

void clearBuffer(void);
