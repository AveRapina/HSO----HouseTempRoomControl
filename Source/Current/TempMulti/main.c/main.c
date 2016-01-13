/*
 * main.c
 *
 * Created: 07.01.2016 23:56:12
 *  Author: HSO
 */ 


#define F_CPU 16000000UL

#include <avr/io.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include "libs/utils/bit_tools.h"
#include "libs/utils/my_utils.h"
#include "libs/lcds/hd774x.h"
#include "libs/interrupts/interruptvectors.h"
#include "libs/timer/timer_utils.h"
#include "libs/ds182x/ds18b20.h"
#include "libs/usart/usart.h"

/************************************************************************/
/* Local Prototipes                                                                     */
/************************************************************************/
void setDeltaMinTemp(uint8_t val);
void setSolMinTemp(uint8_t val);
void setRadMinTemp(uint8_t val);
void setAqsMaxTemp(uint8_t val);
void setRecMinTemp(uint8_t val);
uint8_t checkTempErrorPump1(void);
uint8_t checkTempErrorPump2(void);
uint8_t checkTempErrorPump3(void);




void stateMachine(uint8_t inCode);
uint8_t decodeButton(uint8_t button);
uint8_t debounceKey(uint8_t codeNew);

void updateLcd(void);
void setLcdInitialFields(void);
void showLcdSavedMessage(void);
void showLcdSplash(void);


uint8_t readSwitches(void);
void initGPIO(void);
uint8_t readButtons(void);
void decodeSwitchesStatus(uint8_t val, uint8_t *pump1, uint8_t *pump2, uint8_t *pump3);
void setOutputRelay(uint8_t currentStatusPump1,uint8_t currentStatusPump2,uint8_t currentStatusPump3);



void paramLoadDefaultParameters(void);
void eepromSetDefaultParameters(void);
void paramLoadFromEeprom(void);
void paramSavetoEeprom(void);




// LEDS state
#define GPIO_PORT PORTF

#define LED_RUN_ON bit_clear(GPIO_PORT,0)
#define LED_RUN_OFF bit_set(GPIO_PORT,0)

#define LED_PROG_ON bit_clear(GPIO_PORT,1)
#define LED_PROG_OFF bit_set(GPIO_PORT,1)





// Debug stuff
//#define MAIN_DEBUG
char debugBuffer[10];



// eeproms strucst
typedef struct{
	uint8_t initEeprom;
	
	double setPointRecMinTemp;
	double setPointAqsMaxTemp;
	double setPointRadMinTemp;
	double setPointSolarMinTemp;
	
	double setPointDeltaTemp;
	
	
}eestruct_t;


eestruct_t EEMEM eestruct_eemem;
eestruct_t eestruct_var;



/////////////////////////////////////////////////////////////
////////////////// SCHED ZONE ////////////////////////////////
/////////////////////////////////////////////////////////////

#define SCHEDULER_PRESCALER TIMER0_PRESC128
#define SCHEDULER_RELOAD 31*4 // 1ms tick

// base at 1ms
// task periods
volatile uint16_t taskReadButtonsPeriod =100;
volatile uint16_t taskUpdateLCDPeriod = 500;
volatile uint16_t taskControlPeriod =1000;

//task flags
volatile uint8_t flagTaskReadButtons =0;
volatile uint8_t flagTaskUpdateLcd =0;
volatile uint8_t flagTaskControl =0;

uint8_t flagSaveParametersEeprom=0;

// @ setuo scheulder
void schedulerInit(void){
	TCCR0 |= SCHEDULER_PRESCALER;
	TCCR0 |= TIMER0_WAVEFORM_MODE_CTC;
	OCR0  = SCHEDULER_RELOAD; // timer count reaload
	TIMSK |= (1<< OCIE0); // Enable timer compare interrupt
	
	
	
}

/////////////////////////////////////////////////////////////
////////////////// STATE ZONE ////////////////////////////////
/////////////////////////////////////////////////////////////


// STATE MACHINE
#define STATE_IDLE 0
#define STATE_PROGRAM_SETPOINT_REC 1
#define STATE_PROGRAM_SETPOINT_AQS 2
#define STATE_PROGRAM_SETPOINT_RAD 3
#define STATE_PROGRAM_SETPOINT_SOL 4
#define STATE_PROGRAM_SETPOINT_DELTA 5

#define STATE_SAVE 8


#define BUTTON_ESC 1
#define BUTTON_ENTER 2
#define BUTTON_UP 3
#define BUTTON_DOWN 4



/************************************************************************/
/* @decode Button                                                                     */
/************************************************************************/
uint8_t decodeButton(uint8_t button){
	button &=0x0F; // ensure
	switch (button){
		
		case 1: return BUTTON_ENTER;
		break;
		
		case 2: return BUTTON_UP;
		break;
		
		case 4: return BUTTON_DOWN;
		break;
		
		case 8: return BUTTON_ESC;
		
		
		default: return 0;
		break;
	}
}



#define N_DEBOUNCE 3
/************************************************************************/
/* @debounce function                                                                     */
/************************************************************************/
uint8_t debounceKey(uint8_t codeNew){
	uint8_t key =0; // by default
	static codeOld;
	static keyCount;
	
	// ALREADY SOMETHIN PRESSED
	if(keyCount != 0){
		
		// IF SAME KEY and inside debounce times save
		if(codeNew == codeOld && keyCount <N_DEBOUNCE){ // ONLY IF EQUAL AND DEBOUNCE AVAILABLE
			codeOld =codeNew;
			keyCount++;
			// Reached debounce value and valid key
			if (keyCount == N_DEBOUNCE){
				key = codeNew; // ONLY HERE key is changed;
				
			}
		}
		
	}

	
	// INITIAL CONDITION
	if (keyCount == 0){
		codeOld = codeNew;
		keyCount++;
	}
	
	// if pressed key different reset (user must release the key for new run)
	if(codeNew != codeOld){
		codeOld =codeNew;
		keyCount =1;
	}
	return key;
}



/************************************************************************/
/* @sate machine                                                                     */
/************************************************************************/
void stateMachine(uint8_t inCode){
	
	static uint8_t state;

	//inCode &= 0x07; // ensure clean
	
	
	
	// Switch to the state
	switch (state){
		
		case STATE_IDLE:
			#ifdef MAIN_DEBUG
			USART1_sendStr("IDLE\n\r");
			#endif
			LED_PROG_OFF;
			if(inCode == BUTTON_ENTER) state = STATE_PROGRAM_SETPOINT_REC;
		break;
		
		case STATE_PROGRAM_SETPOINT_REC:
			#ifdef MAIN_DEBUG
			USART1_sendStr("SETPOINT_REC\n\r");
			#endif
			LED_PROG_ON;
			if(inCode==BUTTON_UP)setRecMinTemp(1);
			if(inCode==BUTTON_DOWN)setRecMinTemp(0);
			if(inCode==BUTTON_ENTER)state=STATE_PROGRAM_SETPOINT_AQS;
			if(inCode== BUTTON_ESC)state=STATE_SAVE;
		break;
		
		case STATE_PROGRAM_SETPOINT_AQS:
			#ifdef MAIN_DEBUG
			USART1_sendStr("SETPOINT_AQS\n\r");
			#endif
			if(inCode==BUTTON_UP)setAqsMaxTemp(1);
			if(inCode==BUTTON_DOWN)setAqsMaxTemp(0);
			if(inCode==BUTTON_ENTER)state=STATE_PROGRAM_SETPOINT_RAD;
			if(inCode== BUTTON_ESC)state=STATE_SAVE;
		break;
		
		
		case STATE_PROGRAM_SETPOINT_RAD:
			#ifdef MAIN_DEBUG
			USART1_sendStr("SETPOINT_RAD\n\r");
			#endif
			if(inCode==BUTTON_UP)setRadMinTemp(1);
			if(inCode==BUTTON_DOWN)setRadMinTemp(0);
			if(inCode==BUTTON_ENTER)state=STATE_PROGRAM_SETPOINT_SOL;
			if(inCode== BUTTON_ESC)state=STATE_SAVE;
		break;
		
		
		case STATE_PROGRAM_SETPOINT_SOL:
			#ifdef MAIN_DEBUG
			USART1_sendStr("SETPOINT_SOL\n\r");
			#endif
			if(inCode==BUTTON_UP)setSolMinTemp(1);
			if(inCode==BUTTON_DOWN)setSolMinTemp(0);
			if(inCode==BUTTON_ENTER)state=STATE_SAVE;//STATE_PROGRAM_SETPOINT_DELTA;
			if(inCode== BUTTON_ESC)state=STATE_SAVE;
		break;
		
		
		case STATE_PROGRAM_SETPOINT_DELTA:
			#ifdef MAIN_DEBUG
			USART1_sendStr("SETPOINT_SOL\n\r");
			#endif
			if(inCode==BUTTON_UP)setDeltaMinTemp(1);
			if(inCode==BUTTON_DOWN)setDeltaMinTemp(0);
			if(inCode==BUTTON_ENTER)state=STATE_SAVE;
			if(inCode== BUTTON_ESC)state=STATE_SAVE;
		break;
		
		
		case STATE_SAVE:
			#ifdef MAIN_DEBUG
			USART1_sendStr("SAVE\n\r");
			#endif
			flagSaveParametersEeprom++;
			//wait to be saved and exit
			state=STATE_IDLE;
		break;
		
		default:
		state=STATE_IDLE;
		break;
	}
}




/////////////////////////////////////////////////////////////
////////////////// CTRL ZONE ////////////////////////////////
/////////////////////////////////////////////////////////////
#define TEMP_REC_SET_MIN_DEFAULT 40.0
#define TEMP_AQS_SET_MAX_DEFAULT 90.0
#define TEMP_RADIATOR_MIN_DEFAULT 50.0
#define TEMP_SOLAR_SET_MIN_DEFAULT 40.0

#define TEMP_DELTA_DEFAULT 3.0


double currentRec=80.5;
double currentAqs=20.5;
double currentRad=45.6;
double currentSol=96.5;

// pump status
uint8_t currentStatusPump1=0;
uint8_t currentStatusPump2=0;
uint8_t currentStatusPump3=0;

double currentSetPointRecMin;
double currentSetPointAqsMax;
double currentSetPointRadMin;
double currentSetPointSolMin;

double currentSetPointDelta; // equal for all

uint8_t pump1En,pump2En, pump3En;


#define TEMP_UP_LIMIT 200.0
#define TEMP_LOW_LIMIT -200.0
#define DELTA_STEP 1.0
#define TEMP_STEP 1.0


/************************************************************************/
/* @check erros and return state                                                                     */
/************************************************************************/
uint8_t checkTempErrorPump1(void){
	
	static uint8_t bombaRecAqsState;
	
	
	if(currentRec > (currentSetPointRecMin + currentSetPointDelta/2) && currentAqs < (currentSetPointAqsMax - currentSetPointDelta/2) && (currentRec > currentAqs + currentSetPointDelta/2) && currentAqs < (currentSetPointAqsMax-currentSetPointDelta/2) ) bombaRecAqsState=1;
	
	if(currentRec < (currentSetPointRecMin - currentSetPointDelta/2) || currentRec <(currentAqs - currentSetPointDelta/2 ) || currentAqs >(currentSetPointAqsMax+currentSetPointDelta/2) )bombaRecAqsState=0;
	
	return bombaRecAqsState;
}


/************************************************************************/
/* @check erros RAD and return state                                                                     */
/************************************************************************/
uint8_t checkTempErrorPump2(void){
	
	static uint8_t bombaRecRadState;
	
	
	if(currentRec >(currentSetPointRecMin + currentSetPointDelta/2) && (currentRec > currentSetPointRadMin - currentSetPointDelta/2)) bombaRecRadState=1;
	
	if(currentRec < (currentSetPointRecMin - currentSetPointDelta/2) || currentRec <currentSetPointRadMin -currentSetPointDelta/2) bombaRecRadState=0;
	
	return bombaRecRadState;
}


/************************************************************************/
/* @check erros and return state                                                                     */
/************************************************************************/
uint8_t checkTempErrorPump3(void){
	
	static uint8_t bombaSolAqsState;
	
	
	if(currentSol >(currentSetPointSolMin + currentSetPointDelta/2) && (currentAqs < currentSetPointAqsMax - currentSetPointDelta/2) && currentSol > (currentAqs + currentSetPointDelta/2) && currentAqs < (currentSetPointAqsMax-currentSetPointDelta/2)) bombaSolAqsState=1;
	
	if(currentSol < (currentSetPointSolMin - currentSetPointDelta/2) || currentSol <(currentAqs - currentSetPointDelta/2 ) || currentAqs >(currentSetPointAqsMax+currentSetPointDelta/2) )bombaSolAqsState=0;
	
	return bombaSolAqsState;
}




/************************************************************************/
/* @set rec val                                                                     */
/************************************************************************/
void setRecMinTemp(uint8_t val){
	
	if(val)	currentSetPointRecMin +=1.0;
	else currentSetPointRecMin-=1.0;
	if(currentSetPointRecMin >TEMP_UP_LIMIT) currentSetPointRecMin=TEMP_UP_LIMIT;
	if(currentSetPointRecMin <TEMP_LOW_LIMIT)currentSetPointRecMin=TEMP_LOW_LIMIT;
	
	
}

/************************************************************************/
/* @set aqs max temp                                                                     */
/************************************************************************/
void setAqsMaxTemp(uint8_t val){
	
	if(val)	currentSetPointAqsMax +=TEMP_STEP;
	else currentSetPointAqsMax-=TEMP_STEP;
	if(currentSetPointAqsMax >TEMP_UP_LIMIT) currentSetPointAqsMax=TEMP_UP_LIMIT;
	if(currentSetPointAqsMax <TEMP_LOW_LIMIT)currentSetPointAqsMax=TEMP_LOW_LIMIT;
	
}

/************************************************************************/
/* @set rad max temp                                                                     */
/************************************************************************/
void setRadMinTemp(uint8_t val){
	
	if(val)	currentSetPointRadMin +=TEMP_STEP;
	else currentSetPointRadMin-=TEMP_STEP;
	if(currentSetPointRadMin >TEMP_UP_LIMIT) currentSetPointRadMin=TEMP_UP_LIMIT;
	if(currentSetPointRadMin <TEMP_LOW_LIMIT)currentSetPointRadMin=TEMP_LOW_LIMIT;
	
	
}

/************************************************************************/
/* @set Sol max temp                                                                     */
/************************************************************************/
void setSolMinTemp(uint8_t val){
	
	if(val)	currentSetPointSolMin +=TEMP_STEP;
	else currentSetPointSolMin-=TEMP_STEP;
	if(currentSetPointSolMin >TEMP_UP_LIMIT) currentSetPointSolMin=TEMP_UP_LIMIT;
	if(currentSetPointSolMin <TEMP_LOW_LIMIT)currentSetPointSolMin=TEMP_LOW_LIMIT;
	
	
}


/************************************************************************/
/* @set Sol max temp                                                                     */
/************************************************************************/
void setDeltaMinTemp(uint8_t val){
	
	if(val)	currentSetPointDelta +=DELTA_STEP;
	else currentSetPointDelta-=DELTA_STEP;
	if(currentSetPointDelta<0.5) currentSetPointDelta=DELTA_STEP; // ensure always positive
	
}



/////////////////////////////////////////////////////////////
////////////////// GIO ZONE /////////////////////////////////
/////////////////////////////////////////////////////////////
#define LED_PUMP1_ON bit_clear(GPIO_PORT,2)
#define LED_PUMP1_OFF bit_set(GPIO_PORT,2)

#define LED_PUMP2_ON bit_clear(GPIO_PORT,3)
#define LED_PUMP2_OFF bit_set(GPIO_PORT,3)

#define LED_PUMP3_ON bit_clear(GPIO_PORT,4)
#define LED_PUMP3_OFF bit_set(GPIO_PORT,4)

#define RELAY_PUMP1_ON bit_set(GPIO_PORT,5)
#define RELAY_PUMP1_OFF bit_clear(GPIO_PORT,5)

#define RELAY_PUMP2_ON bit_set(GPIO_PORT,6)
#define RELAY_PUMP2_OFF bit_clear(PORTB,6)

#define RELAY_PUMP3_ON bit_set(GPIO_PORT,7)
#define RELAY_PUMP3_OFF bit_clear(GPIO_PORT,7)


/************************************************************************/
/* @read the buttons and complement and clean                                                                     */
/************************************************************************/
uint8_t readButtons(void){

	return (~PINA) &0x0F; // read the low nibble
}



#define PUMP1_CODE 1
#define PUMP2_CODE 2
#define PUMP3_CODE 4
void decodeSwitchesStatus(uint8_t val, uint8_t *pump1, uint8_t *pump2, uint8_t *pump3){
	if((val &PUMP1_CODE)>0)*pump1=1;
	else *pump1=0;
	
	if((val & PUMP2_CODE)>0)*pump2=1;
	else *pump2=0;
	
	if((val & PUMP3_CODE)>0)*pump3=1;
	else *pump3=0;
	
}

/************************************************************************/
/* @read the enable swichches                                                                     */
/************************************************************************/
uint8_t readSwitches(void){
	
	return ((~PINA) & 0xF0)>>4;
}

/************************************************************************/
/* @set the output relay                                                                     */
/************************************************************************/
void setOutputRelay(uint8_t currentStatusPump1,uint8_t currentStatusPump2,uint8_t currentStatusPump3){
	
	// pump 1
	if(currentStatusPump1){
		RELAY_PUMP1_ON;
		LED_PUMP1_ON;
	}else{
		RELAY_PUMP1_OFF;
		LED_PUMP1_OFF;
	}
	
	// pump2
	if(currentStatusPump2){
		RELAY_PUMP2_ON;
		LED_PUMP2_ON;
		}else{
		RELAY_PUMP2_OFF;
		LED_PUMP2_OFF;
	}
	
	
	// pump2
	if(currentStatusPump3){
		RELAY_PUMP3_ON;
		LED_PUMP3_ON;
		}else{
		RELAY_PUMP3_OFF;
		LED_PUMP3_OFF;
	}
	
}


/************************************************************************/
/* @init the gpio                                                                     */
/************************************************************************/
void initGPIO(void){
	DDRA = 0x00;
	PORTA = 0xFF; // enable pulls ups
	
	DDRF = 0b11111111;
	PORTF = 0x00;

}




/////////////////////////////////////////////////////////////
////////////////// EEPROM ZONE //////////////////////////////
/////////////////////////////////////////////////////////////

/************************************************************************/
/* @load hardcodded default values                                                                     */
/************************************************************************/
void paramLoadDefaultParameters(void){
	currentSetPointRecMin = TEMP_REC_SET_MIN_DEFAULT;
	currentSetPointAqsMax = TEMP_AQS_SET_MAX_DEFAULT;
	currentSetPointRadMin = TEMP_RADIATOR_MIN_DEFAULT;
	currentSetPointSolMin = TEMP_SOLAR_SET_MIN_DEFAULT;
	
	currentSetPointDelta = TEMP_DELTA_DEFAULT; // equal for all
	
}


/************************************************************************/
/* @ set initial values to eeprom  if nothin there yet                                                                   */
/************************************************************************/
void eepromSetDefaultParameters(void){
	eestruct_var.initEeprom=1; // emprom init
	
	eestruct_var.setPointRecMinTemp = TEMP_REC_SET_MIN_DEFAULT;
	eestruct_var.setPointAqsMaxTemp = TEMP_AQS_SET_MAX_DEFAULT;
	eestruct_var.setPointRadMinTemp = TEMP_REC_SET_MIN_DEFAULT;
	eestruct_var.setPointSolarMinTemp = TEMP_AQS_SET_MAX_DEFAULT;
	
	eestruct_var.setPointDeltaTemp = TEMP_DELTA_DEFAULT;
		
	
	
	eeprom_write_block((const void*)&eestruct_var,(void*)&eestruct_eemem,sizeof(eestruct_t));
	
}


/************************************************************************/
/* @ load eeprom saved values                                                                     */
/************************************************************************/
void paramLoadFromEeprom(void){
	//uint8_t temp=0;
	// read from emprom
	eeprom_read_block((void*)&eestruct_var, (const void*)&eestruct_eemem,sizeof(eestruct_t));
	
	// test the fits field to check if it was written else use default and load
	if((eestruct_var.initEeprom &0xFF) ==0xFF){
		eepromSetDefaultParameters();
		paramLoadDefaultParameters();
		
	}
	else{
		// write to the global variables
		currentSetPointRecMin = eestruct_var.setPointRecMinTemp;
		currentSetPointAqsMax = eestruct_var.setPointAqsMaxTemp;
		currentSetPointRadMin = eestruct_var.setPointRadMinTemp;
		currentSetPointSolMin = eestruct_var.setPointSolarMinTemp;
		

		currentSetPointDelta = eestruct_var.setPointDeltaTemp; // equal for all
		
	}
	
	
	
}


/************************************************************************/
/* @save current values to eerprom                                                                     */
/************************************************************************/
void paramSavetoEeprom(void){
	// save paramenetrs on the run
	eestruct_var.initEeprom=1; // emprom init
	
	eestruct_var.setPointRecMinTemp = currentSetPointRecMin;
	eestruct_var.setPointAqsMaxTemp = currentSetPointAqsMax;
	eestruct_var.setPointRadMinTemp = currentSetPointRadMin;
	eestruct_var.setPointSolarMinTemp = currentSetPointSolMin;
	
	eestruct_var.setPointDeltaTemp = currentSetPointDelta;
	
	// save block
	eeprom_write_block((const void*)&eestruct_var,(void*)&eestruct_eemem,sizeof(eestruct_t));
	
}




/////////////////////////////////////////////////////////////
////////////////// LCD ZONE /////////////////////////////////
/////////////////////////////////////////////////////////////

/************************************************************************/
/* @show initial splash                                                                     */
/************************************************************************/
void showLcdSplash(void){
	
	LCD_gotoXY(4,0);
	LCD_sendString("Room Control");
	LCD_gotoXY(3,1);
	LCD_sendString("Serafim Cunha");
	_delay_ms(2000);
	LCD_clr();
	
	
	
}

// Current: 20.0º State: OFF
// SetPoint: 30.0 Hist: 10.5
//

/************************************************************************/
/* @set lcd initail fields                                                                     */
/************************************************************************/
void setLcdInitialFields(void){

	LCD_clr();
	LCD_gotoXY(2,0);
	LCD_sendString("REC");
	LCD_gotoXY(6,0);
	LCD_sendString("AQS");
	LCD_gotoXY(11,0);
	LCD_sendString("RAD");
	LCD_gotoXY(16,0);
	LCD_sendString("SOL");
	
	// 4 field filds
	LCD_gotoXY(0,3);
	LCD_sendString("ST");
	
	LCD_gotoXY(3,3);
	LCD_sendString("1:");
	LCD_gotoXY(9,3);
	LCD_sendString("2:");
	
	LCD_gotoXY(15,3);
	LCD_sendString("3:");
	


}

/************************************************************************/
/* @show saved messsage on lcd                                                                      */
/************************************************************************/
void showLcdSavedMessage(void){
	
	LCD_clr();
	LCD_gotoXY(7,0);
	LCD_sendString("SAVED");
	
	LCD_gotoXY(7,1);
	LCD_sendString("PARAMETERS");

	_delay_ms(500);
	
}

/************************************************************************/
/* @ convert double two parts to print                                                                     */
/************************************************************************/
void splitDouble(double *d, int precision, int *intPart, int *decPart ){
	*intPart = (int)*d;
	*decPart = (int)( (*d - *intPart) *pow(10, precision) + 0.5);
	
}

/************************************************************************/
/* @update lcd with current vars                                                                     */
/************************************************************************/
void updateLcd(void){
	char buffer[20];
	int integerPart[4];
	int decimalPart[4];
	// RELAY
	LCD_gotoXY(5,3);
	if(pump1En)LCD_sendString("ON ");
	else LCD_sendString("OFF");
	
	LCD_gotoXY(11,3);
	if(pump2En)LCD_sendString("ON ");
	else LCD_sendString("OFF");
	
	LCD_gotoXY(17,3);
	if(pump3En)LCD_sendString("ON ");
	else LCD_sendString("OFF");
	
	
	// current temp
	LCD_gotoXY(0,1);
	splitDouble(&currentRec,1,&integerPart[0],&decimalPart[0]);
	splitDouble(&currentAqs,1,&integerPart[1],&decimalPart[1]);		
	splitDouble(&currentRad,1,&integerPart[2],&decimalPart[2]);
	splitDouble(&currentSol,1,&integerPart[3],&decimalPart[3]);
	
	sprintf(buffer,"%2d.%1d %2d.%1d %2d.%1d %2d.%1d",integerPart[0], decimalPart[0],integerPart[1], decimalPart[1],integerPart[2], decimalPart[2],integerPart[3], decimalPart[3]);
	LCD_sendString(buffer);
	
	
	// setpoints
	LCD_gotoXY(0,2);
	splitDouble(&currentSetPointRecMin,1,&integerPart[0],&decimalPart[0]);
	splitDouble(&currentSetPointAqsMax,1,&integerPart[1],&decimalPart[1]);
	splitDouble(&currentSetPointRadMin,1,&integerPart[2],&decimalPart[2]);
	splitDouble(&currentSetPointSolMin,1,&integerPart[3],&decimalPart[3]);
	
	sprintf(buffer,"%2d.%1d %2d.%1d %2d.%1d %2d.%1d",integerPart[0], decimalPart[0],integerPart[1], decimalPart[1],integerPart[2], decimalPart[2],integerPart[3], decimalPart[3]);
	LCD_sendString(buffer);
	
	

}

/************************************************************************/
/* @read temperatures                                                                     */
/************************************************************************/
void readTemperatures(void){
	static aqs;
	if(currentAqs <40.0){
		aqs =1;
	}else if(currentAqs>95.0) aqs=0;
	
	if(aqs)currentAqs+=1.0;
	else currentAqs -=1.0;
	
	/*
	static sol;
	if(currentSol <40.0){
		sol =1;
	}else if(currentSol>80.0) sol=0;
	
	if(aqs)currentSol+=1.0;
	else currentSol -=1.0;
	*/
}


int main(void){
    
	initGPIO();
	LCD_init();
	
	showLcdSplash();
	setLcdInitialFields();
	paramLoadFromEeprom();
	
	USART1_config(USART1_MY_UBBRN,USART_DATA_FORMAT_8BITS|USART_STOP_BITS_1,USART_TRANSMIT_ENABLE|USART_RECEIVE_ENABLE| USART_INTERRUPT_ENABLE);
	
	USART1_sendStr("Hello");
	
	schedulerInit();
	
	// check for the default values
	
	sei(); //enable interrups
	
	// loop while
	
	while(1){
		
		
		// cintrol zone
		if(flagTaskControl){
			
			LED_RUN_ON;
			readTemperatures();
			//currentTemp -=1.0;
			_delay_ms(50);
			LED_RUN_OFF;
			
			
			
			uint8_t switchesVal = readSwitches();
			
			
			decodeSwitchesStatus(switchesVal, &pump1En, &pump2En, &pump3En);
			
			currentStatusPump1 = checkTempErrorPump1()&pump1En; // chec the out
			currentStatusPump2 = checkTempErrorPump2()&pump2En;
			currentStatusPump3 = checkTempErrorPump3()&pump3En;
			
			
			setOutputRelay(currentStatusPump1,currentStatusPump2,currentStatusPump3);
			flagTaskControl=0;
		}
		
		// user bottons area
		if(flagTaskReadButtons){
			uint8_t portVal = readButtons();
			uint8_t code = decodeButton(portVal);
			code = debounceKey(code);
			
			#ifdef MAIN_DEBUG
			sprintf(debugBuffer,"Key %d",code);
			USART1_sendStr(debugBuffer);
			#endif
			
			stateMachine(code); // go to machine
			
			flagTaskReadButtons=0;
		}
		
		
		// lcd update area
		if(flagTaskUpdateLcd){
			
			//showLcdSavedMessage();
			updateLcd(); // update the lcd
			
			flagTaskUpdateLcd=0;
		}
		
		// save to eeprom
		if(flagSaveParametersEeprom){
			
			paramSavetoEeprom(); // save value sto eeprom
			showLcdSavedMessage();
			setLcdInitialFields();
			updateLcd();
			flagSaveParametersEeprom=0;
		}
		
	}
}