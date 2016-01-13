#include <avr/io.h>




/************************************************************************/
/* TIMER0 REG-> TCCR0 CLOCK PRESCALERS INTERNAL                                         */
/************************************************************************/
#define TIMER0_NO_CLK 0x00
#define TIMER0_PRESC1  (1<<CS00)
#define TIMER0_PRESC8  (1<<CS01)
#define TIMER0_PRESC32 (1<<CS01) |  (1<<CS00)
#define TIMER0_PRESC64  (1<<CS02)
#define TIMER0_PRESC128 (1<<CS02) | (1<<CS00)
#define TIMER0_PRESC256  (1<<CS02) | (1<<CS01)
#define TIMER0_PRESC1024 (1<<CS02) | (1<<CS01) | (1<<CS00)

/************************************************************************/
/* TIMER0 REG-> TCCR0 WAVEFORM MODE                                       */
/************************************************************************/
#define TIMER0_WAVEFORM_MODE_NORMAL 0x00
#define TIMER0_WAVEFORM_MODE_PWM_PHASE_CRR (1<<WGM00)
#define TIMER0_WAVEFORM_MODE_CTC (1<<WGM01)
#define TIMER0_WAVEFORM_MODE_PWM_FAST (1<<WGM01) | (1<<WGM00)

/************************************************************************/
/* TIMER0 REG-> TCCR0 OUTPUT MODE                                       */
/************************************************************************/
#define TIMER0_OUTPUT_MODE_NORMAL 0x00
#define TIMER0_OUTPUT_MODE_CTC_TOGGLE_PWM_RESERVED (1<<COM00)
#define TIMER0_OUTPUT_MODE_CTC_CLEAR_PWM_NON_INV (1<<COM01)
#define TIMER0_OUTPUT_MODE_CTC_SET_PWM_INV (1<<COM01) | (1<<COM00)



/************************************************************************/
/* TIMER1 REG-> TCCR1A CLOCK PRESCALERS INTERNAL                                       */
/************************************************************************/
#define TIMER1_NO_CLK 0x00
#define TIMER1_PRESC1  (1<<CS00)
#define TIMER1_PRESC8  (1<<CS01)
#define TIMER1_PRESC64  (1<<CS01) |  (1<<CS00)
#define TIMER1_PRESC256  (1<<CS02)
#define TIMER1_PRESC1024 (1<<CS02) | (1<<CS00)

/************************************************************************/
/* TIMER1 REG-> TCCR1A CLOCK SOURCE EXTERNAL                                      */
/************************************************************************/
#define TIMER1_EXT_CLK_EDGE_FALL (1<<CS02) | (1<<CS01)
#define TIMER1_EXT_CLK_EDGE_RISE (1<<CS02) | (1<<CS01) | (1<<CS00)


/************************************************************************/
/* TIMER1 REG-> TCCR1x WAVEFORM MODE                                       */
/************************************************************************/
#define TIMER1_WAVEFORM_MODE_NORMAL 0x00									//NORMAL
#define TIMER1_WAVEFORM_MODE_PWM_PHASE_CORR_8BIT (1<<WGM10)				//PWM PHASE CORRECT 8 BIT
#define TIMER1_WAVEFORM_MODE_PWM_PHASE_CORR_9BIT (1<<WGM11)				//PWM PHASE CORRECT 9 BIT
#define TIMER1_WAVEFORM_MODE_PWM_PHASE_CORR_10BIT (1<<WGM10) | (1<<WGM11)	//PWM PHASE CORRECT 10 BIT
#define	TIMER1_WAVEFORM_MODE_CTC_OCRA (1<<WGM12)								//CTC TOP OCRA

#define TIMER1_WAVEFORM_MODE_PWM_FAST_8BIT (1<<WGM12) | (1<<WGM10)			//FAST PWM 8 BIT
#define TIMER1_WAVEFORM_MODE_PWM_FAST_9BIT (1<<WGM12) | (1<<WGM11)			//FAST PWM 9 BIT
#define TIMER1_WAVEFORM_MODE_PWM_FAST_10BIT (1<<WGM12) | (1<<WGM11) | (1<<WGM10)	//FAST PWM 10 BIT
#define TIMER1_WAVEFORM_MODE_PWM_PHASE_FREQ_CORR_ICR (1<<WGM13)				//PWM PHASE AND FREQUENCY CORRECT TOP ICR
//#define TIMER1_WAVEFORM_MODE_PWM_PHASE_FREQ_CORR_OCRA (1<<WGM13) | (1<<WGM10)	//PWM PHASE AND FREQUENCY CORRECT TOP OCRA
//#define TIMER1_WAVEFORM_MODE_PWM_PHASE_CORR_ICR (1<<WGM13) | (1<<WGM11)		//PWM PHASE CORRECT TOP ICR
//#define TIMER1_WAVEFORM_MODE_PWM_PHASE_CORR_OCRA (1<<WGM13) | (1<<WGM11) | (1<<WGM10) //PWM PHASE CORRECT TOP OCRA
#define TIMER1_WAVEFORM_MODE_CTC_ICR (1<<WGM13) | (1<<WGM12)							//CTC TOP ICR
//#define TIMER1_WAVEFORM_MODE_PWM_FAST_ICR (1<<WGM13) | (1<<WGM12) | (1<<WGM11)		//FAST PWM TOP ICR
//#define TIMER1_WAVEFORM_MODE_PWM_FAST_OCRA (1<<WGM13) | (1<<WGM12) | (1<<WGM11) |(WGM10)//FAST PWM TOP OCRA
 //esse por ex o wgm13 nao pode ir | com o wgm10 esse foi o meu erro


/************************************************************************/
/* TIMER1 REG-> TCCR1x OUTPUT MODE                                       */
/************************************************************************/
#define TIMER1_OUTPUT_MODE_NONE 0x00				//NORMAL PORT OPERATION OC1A/B/C DISCONNECTED
// OC1A
#define TIMER1_OUTPUT_MODE_OC1A_COMP_OUTPUT_TOGGLE (1<<COM1A0) //TOGGLE OC1A ON COMPARE MATCH
#define TIMER1_OUTPUT_MODE_OC1A_COMP_OUTPUT_CLEAR (1<<COM1A1) //CLEAR OC1A ON COMPARE MATCH (NON-INVERTING MODE)
#define TIMER1_OUTPUT_MODE_OC1A_COMP_OUTPUT_SET (1<<COM1A1) | (1<<COM1A0) //SET OC1A ON COMPARE MATCH (INVERTING MODE)
// OC1B
#define TIMER1_OUTPUT_MODE_OC1B_COMP_OUTPUT_TOGGLE (1<<COM1B0) //TOGGLE OC1B ON COMPARE MATCH
#define TIMER1_OUTPUT_MODE_OC1B_COMP_OUTPUT_CLEAR (1<<COM1B1) //CLEAR OC1B ON COMPARE MATCH (NON-INVERTING MODE)
#define TIMER1_OUTPUT_MODE_OC1B_COMP_OUTPUT_SET (1<<COM1B1) | (1<<COM1B0) //SET OC1B ON COMPARE MATCH (INVERTING MODE)
// OC1C
#define TIMER1_OUTPUT_MODE_OC1C_COMP_OUTPUT_TOGGLE (1<<COM1C0) //TOGGLE OC1C ON COMPARE MATCH
#define TIMER1_OUTPUT_MODE_OC1C_COMP_OUTPUT_CLEAR (1<<COM1C1) //CLEAR OC1C ON COMPARE MATCH (NON-INVERTING MODE)
#define TIMER1_OUTPUT_MODE_OC1C_COMP_OUTPUT_SET (1<<COM1C1) | (1<<COM1C0) //SET OC1C ON COMPARE MATCH (INVERTING MODE)


/************************************************************************/
/* TIMER1 ENABLE INTERRUPT FLAGS  REG-> TIFR                                      */
/************************************************************************/
#define TIMER1_INT_FLG_INPUTCAPT_EN (1<<ICF1)
#define TIMER1_INT_FLG_OVF_EN (1<<TOV1)
#define TIMER1_INT_FLG_OUTPUT_COMPA_EN (1<<OCF1A)
#define TIMER1_INT_FLG_OUTPUT_COMPB_EN (1<<OCF1B)

/************************************************************************/
/* TIMER1 ENABLE INTERRUPT FLAGS  REG-> ETIFR                                      */
/************************************************************************/
#define TIMER1_INT_FLG_OUTPUT_COMPC_EN (1<<OCF1C)




