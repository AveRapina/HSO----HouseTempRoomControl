/*
 * hd774x_pcf.c
 *
 * Created: 02.09.2015 15:42:24
 *  Author: HSO
 */ 

#include <inttypes.h>
#include <avr/pgmspace.h>

#include "hdpcf774x.h"
#include "../pcf857x/pcf857x.h"


// local Globals
volatile uint8_t dataport=0;

#define lcd_e_delay()   __asm__ __volatile__( "rjmp 1f\n 1:" );
#define lcd_e_toggle()  toggle_e()

#if LCD_LINES==1
#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_4BIT_1LINE 
#else
#define LCD_FUNCTION_DEFAULT    LCD_FUNCTION_4BIT_2LINES 
#endif



/* 
** function prototypes 
*/
static void LCDPCF_toggleE(void);

/*
** local functions
*/

/* toggle Enable Pin to initiate write */
static void LCDPCF_toggleE(void)
{
	PCF_writePin(LCD_PCF_ADDR,LCD_E_PIN,1);
	lcd_e_delay();
	PCF_writePin(LCD_PCF_ADDR,LCD_E_PIN,0);
}
/************************************************************************/
/* 
/*
/*                                                                     */
/************************************************************************/

void LCDPCF_init(void){
	dataport =0;
	PCF_write(LCD_PCF_ADDR, dataport);
	_delay_ms(16);
	
	/* Initial write to lcd is 8bit */
	dataport |= _BV(LCD_DATA1_PIN);
	dataport |= _BV(LCD_DATA0_PIN);
	PCF_write(LCD_PCF_ADDR, dataport);
	
	LCDPCF_toggleE();
	_delay_ms(5);
	
	LCDPCF_toggleE();
	_delay_us(64);
	
	LCDPCF_toggleE();
	_delay_us(64);
	
	dataport &= ~_BV(LCD_DATA0_PIN);
	PCF_write(LCD_PCF_ADDR,dataport);
	LCDPCF_toggleE();
	_delay_us(64);
	
	// from now the lcd accepts 4 bit I/O,
	// send some commands	
	
}




/*************************************************************************
 delay loop for small accurate delays: 16-bit counter, 4 cycles/loop
*************************************************************************/
static inline void _delayFourCycles(unsigned int __count)
{
    if ( __count == 0 )    
        __asm__ __volatile__( "rjmp 1f\n 1:" );    // 2 cycles
    else
        __asm__ __volatile__ (
    	    "1: sbiw %0,1" "\n\t"                  
    	    "brne 1b"                              // 4 cycles/loop
    	    : "=w" (__count)
    	    : "0" (__count)
    	   );
}


/************************************************************************* 
delay for a minimum of <us> microseconds
the number of loops is calculated at compile-time from MCU clock frequency
*************************************************************************/
#define delay(us)  _delayFourCycles( ( ( 1*(F_CPU/4000) )*us)/1000 )



void LCDPCF_gotoXY(uint8_t x, uint8_t y){
	
	register uint8_t DDRAMAddr;
	// remap lines into proper order
	switch(y)
	{
		case 0: DDRAMAddr = LCD_LINE0_DDRAMADDR+x; break;
		case 1: DDRAMAddr = LCD_LINE1_DDRAMADDR+x; break;
		case 2: DDRAMAddr = LCD_LINE2_DDRAMADDR+x; break;
		case 3: DDRAMAddr = LCD_LINE3_DDRAMADDR+x; break;
		default: DDRAMAddr = LCD_LINE0_DDRAMADDR+x;
	}
	
	// set data address
	LCDPCF_sendCommand(1<<LCD_DDRAM | DDRAMAddr);
	
}



/*************************************************************************
Low-level function to write byte to LCD controller
Input:    data   byte to write to LCD
          rs     1: write data    
                 0: write instruction
Returns:  none
*************************************************************************/
static void LCDPCF_writeByte(uint8_t data,uint8_t rs) 
{
	if (rs) /* write data        (RS=1, RW=0) */
		dataport |= _BV(LCD_RS_PIN);
	else /* write instruction (RS=0, RW=0) */
		dataport &= ~_BV(LCD_RS_PIN);
	dataport &= ~_BV(LCD_RW_PIN);
	PCF_write(LCD_PCF_ADDR, dataport);

	/* output high nibble first */
    dataport &= ~_BV(LCD_DATA3_PIN);
    dataport &= ~_BV(LCD_DATA2_PIN);
    dataport &= ~_BV(LCD_DATA1_PIN);
    dataport &= ~_BV(LCD_DATA0_PIN);
	if(data & 0x80) dataport |= _BV(LCD_DATA3_PIN);
	if(data & 0x40) dataport |= _BV(LCD_DATA2_PIN);
	if(data & 0x20) dataport |= _BV(LCD_DATA1_PIN);
	if(data & 0x10) dataport |= _BV(LCD_DATA0_PIN);
	PCF_write(LCD_PCF_ADDR, dataport);
	LCDPCF_toggleE();

	/* output low nibble */
	dataport &= ~_BV(LCD_DATA3_PIN);
	dataport &= ~_BV(LCD_DATA2_PIN);
	dataport &= ~_BV(LCD_DATA1_PIN);
	dataport &= ~_BV(LCD_DATA0_PIN);
	if(data & 0x08) dataport |= _BV(LCD_DATA3_PIN);
	if(data & 0x04) dataport |= _BV(LCD_DATA2_PIN);
	if(data & 0x02) dataport |= _BV(LCD_DATA1_PIN);
	if(data & 0x01) dataport |= _BV(LCD_DATA0_PIN);
	PCF_write(LCD_PCF_ADDR, dataport);
	LCDPCF_toggleE();

	/* all data pins high (inactive) */
	dataport |= _BV(LCD_DATA0_PIN);
	dataport |= _BV(LCD_DATA1_PIN);
	dataport |= _BV(LCD_DATA2_PIN);
	dataport |= _BV(LCD_DATA3_PIN);
	PCF_write(LCD_PCF_ADDR, dataport);
}


/*************************************************************************
Low-level function to read byte from LCD controller
Input:    rs     1: read data    
                 0: read busy flag / address counter
Returns:  byte read from LCD controller
*************************************************************************/
static uint8_t LCDPCF_readByte(uint8_t rs) 
{
    uint8_t data;

    if (rs) /* write data        (RS=1, RW=0) */
    	dataport |= _BV(LCD_RS_PIN);
    else /* write instruction (RS=0, RW=0) */
    	dataport &= ~_BV(LCD_RS_PIN);
    dataport |= _BV(LCD_RW_PIN);
    PCF_write(LCD_PCF_ADDR, dataport);

    PCF_writePin(LCD_PCF_ADDR,LCD_E_PIN,1);
	lcd_e_delay();
	data |= PCF_readPin(LCD_PCF_ADDR, LCD_DATA0_PIN) << 4;
	PCF_writePin(LCD_PCF_ADDR, LCD_E_PIN,0);

	lcd_e_delay();                       /* Enable 500ns low       */

	PCF_writePin(LCD_PCF_ADDR,LCD_E_PIN,1);
	lcd_e_delay();
	data |= PCF_readPin(LCD_PCF_ADDR, LCD_DATA0_PIN) &0x0F;    /* read low nibble        */
	PCF_writePin(LCD_PCF_ADDR, LCD_E_PIN,0);

    return data;
}


/*************************************************************************
loops while lcd is busy, returns address counter
*************************************************************************/
static uint8_t LCDPCF_waitBusy(void)

{
    register uint8_t c;
    
    /* wait until busy flag is cleared */
    while ( (c=LCDPCF_readByte(0)) & (1<<LCD_BUSY)) {}
    
    /* the address counter is updated 4us after the busy flag is cleared */
    delay(2);

    /* now read the address counter */
    return (LCDPCF_readByte(0));  // return address counter
    
}/* lcd_waitbusy */


/*************************************************************************
Move cursor to the start of next line or to the first line if the cursor 
is already on the last line.
*************************************************************************/
static inline void LCDPCF_newLine(uint8_t pos){
register uint8_t addressCounter;


#if LCD_LINES==1
    addressCounter = 0;
#endif
#if LCD_LINES==2
    if ( pos < (LCD_START_LINE2) )
        addressCounter = LCD_START_LINE2;
    else
        addressCounter = LCD_START_LINE1;
#endif
#if LCD_LINES==4
    if ( pos < LCD_START_LINE3 )
        addressCounter = LCD_START_LINE2;
    else if ( (pos >= LCD_START_LINE2) && (pos < LCD_START_LINE4) )
        addressCounter = LCD_START_LINE3;
    else if ( (pos >= LCD_START_LINE3) && (pos < LCD_START_LINE2) )
        addressCounter = LCD_START_LINE4;
    else 
        addressCounter = LCD_START_LINE1;
#endif
	LCDPCF_sendCommand((1<<LCD_DDRAM)+addressCounter);

}/* lcd_newline */


/*
** PUBLIC FUNCTIONS 
*/

/*************************************************************************
Send LCD controller instruction command
Input:   instruction to send to LCD controller, see HD44780 data sheet
Returns: none
*************************************************************************/
void LCDPCF_sendCommand(uint8_t cmd)
{
    LCDPCF_waitBusy();
    LCDPCF_writeByte(cmd,0);
}


/*************************************************************************
Send data byte to LCD controller 
Input:   data to send to LCD controller, see HD44780 data sheet
Returns: none
*************************************************************************/
void LCDPCF_sendData(uint8_t data)
{
    LCDPCF_waitBusy();;
    LCDPCF_writeByte(data,1);
}





/*************************************************************************
*************************************************************************/
int LCDPCF_getXY(void)
{
    return LCDPCF_waitBusy();
}


/*************************************************************************
Clear display and set cursor to home position
*************************************************************************/
void LCDPCF_clrScr(void)
{
    LCDPCF_sendCommand(1<<LCD_CLR);
}


/*************************************************************************
Set illumination pin
*************************************************************************/
void LCDPCF_ilumination(uint8_t onoff)
{
	if(onoff)
		dataport &= ~_BV(LCD_LED_PIN);
	else
		dataport |= _BV(LCD_LED_PIN);
	PCF_write(LCD_PCF_ADDR, dataport);
}


/*************************************************************************
Set cursor to home position
*************************************************************************/
void LCDPCF_home(void)
{
    LCDPCF_sendCommand(1<<LCD_HOME);
}


/*************************************************************************
Display character at current cursor position 
Input:    character to be displayed                                       
Returns:  none
*************************************************************************/
void LCDPCF_sendChar(char c)
{
    uint8_t pos;

    pos = LCDPCF_waitBusy();   // read busy-flag and address counter
    if (c=='\n')
    {
        LCDPCF_newLine(pos);
    }
    else
    {
#if LCD_WRAP_LINES==1
#if LCD_LINES==1
        if ( pos == LCD_START_LINE1+LCD_DISP_LENGTH ) {
            LCDPCF_writeByte((1<<LCD_DDRAM)+LCD_START_LINE1,0);
        }
#elif LCD_LINES==2
        if ( pos == LCD_START_LINE1+LCD_DISP_LENGTH ) {
            LCDPCF_writeByte((1<<LCD_DDRAM)+LCD_START_LINE2,0);    
        }else if ( pos == LCD_START_LINE2+LCD_DISP_LENGTH ){
            LCDPCF_writeByte((1<<LCD_DDRAM)+LCD_START_LINE1,0);
        }
#elif LCD_LINES==4
        if ( pos == LCD_START_LINE1+LCD_DISP_LENGTH ) {
            LCDPCF_writeByte((1<<LCD_DDRAM)+LCD_START_LINE2,0);    
        }else if ( pos == LCD_START_LINE2+LCD_DISP_LENGTH ) {
            LCDPCF_writeByte((1<<LCD_DDRAM)+LCD_START_LINE3,0);
        }else if ( pos == LCD_START_LINE3+LCD_DISP_LENGTH ) {
            LCDPCF_writeByte((1<<LCD_DDRAM)+LCD_START_LINE4,0);
        }else if ( pos == LCD_START_LINE4+LCD_DISP_LENGTH ) {
            LCDPCF_writeByte((1<<LCD_DDRAM)+LCD_START_LINE1,0);
        }
#endif
		LCDPCF_waitBusy();
#endif
        LCDPCF_writeByte(c, 1);
    }

}


/*************************************************************************
Display string without auto linefeed 
Input:    string to be displayed
Returns:  none
*************************************************************************/
void LCDPCF_sendString(const char *s)
/* print string on lcd (no auto linefeed) */
{
    register char c;

    while ( (c = *s++) ) {
        LCDPCF_sendChar(c);
    }

}


/*************************************************************************
Display string from program memory without auto linefeed 
Input:     string from program memory be be displayed                                        
Returns:   none
*************************************************************************/
void lcd_puts_p(const char *progmem_s)
/* print string from program memory on lcd (no auto linefeed) */
{
    register char c;

    while ( (c = pgm_read_byte(progmem_s++)) ) {
        LCDPCF_sendChar(c);
    }

}/* lcd_puts_p */



