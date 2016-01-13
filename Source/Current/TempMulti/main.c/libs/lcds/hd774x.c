#include "hd774x.h"

#ifndef F_CPU
#define F_CPU 2000000UL
#endif

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <string.h>


/*
TODO LIST
1 - Check for LCD_home not working.
2 - Check delays and change for ready flag check of the lcd 
*/

const uint8_t LcdCustomChar[] PROGMEM=//define 8 custom LCD chars
{
	0x00, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, // 0. 0/5 full progress block
	0x00, 0x1F, 0x10, 0x10, 0x10, 0x10, 0x1F, 0x00, // 1. 1/5 full progress block
	0x00, 0x1F, 0x18, 0x18, 0x18, 0x18, 0x1F, 0x00, // 2. 2/5 full progress block
	0x00, 0x1F, 0x1C, 0x1C, 0x1C, 0x1C, 0x1F, 0x00, // 3. 3/5 full progress block
	0x00, 0x1F, 0x1E, 0x1E, 0x1E, 0x1E, 0x1F, 0x00, // 4. 4/5 full progress block
	0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x00, // 5. 5/5 full progress block
	0x03, 0x07, 0x0F, 0x1F, 0x0F, 0x07, 0x03, 0x00, // 6. rewind arrow
	0x18, 0x1C, 0x1E, 0x1F, 0x1E, 0x1C, 0x18, 0x00  // 7. fast-forward arrow
};



/************************************************************************/
/* @send char to lcd                                                                     */
/************************************************************************/
void LCD_sendChar(uint8_t ch){

#ifdef LCD_4bit
	//4 bit part
	LDP=(ch&0b11110000);
	LCP|=1<<LCD_RS;
	LCP|=1<<LCD_E;		
	_delay_ms(1);
	LCP&=~(1<<LCD_E);	
	LCP&=~(1<<LCD_RS);
	_delay_ms(1);
	LDP=((ch&0b00001111)<<4);
	LCP|=1<<LCD_RS;
	LCP|=1<<LCD_E;		
	_delay_ms(1);
	LCP&=~(1<<LCD_E);	
	LCP&=~(1<<LCD_RS);
	_delay_ms(1);
#else
	//8 bit part
	LDP=ch;
	LCP|=1<<LCD_RS;
	LCP|=1<<LCD_E;		
	_delay_ms(1);
	LCP&=~(1<<LCD_E);	
	LCP&=~(1<<LCD_RS);
	_delay_ms(1);
#endif
}


/************************************************************************/
/* @send comand to lcd                                                                     */
/************************************************************************/
void LCD_sendCommand(uint8_t cmd){
#ifdef LCD_4bit	
	//4 bit part
	LDP=(cmd&0b11110000);
	LCP|=1<<LCD_E;		
	_delay_ms(1);
	LCP&=~(1<<LCD_E);
	_delay_ms(1);
	LDP=((cmd&0b00001111)<<4);	
	LCP|=1<<LCD_E;		
	_delay_ms(1);
	LCP&=~(1<<LCD_E);
	_delay_ms(1);
#else
	//8 bit part
	LDP=cmd;
	LCP|=1<<LCD_E;		
	_delay_ms(1);
	LCP&=~(1<<LCD_E);
	_delay_ms(1);	
#endif
}


/************************************************************************/
/* @init lcd                                                                     */
/************************************************************************/
void LCD_init(void){
#ifdef LCD_4bit	
	//4 bit part
	_delay_ms(15);
	LDP=0x00;
	LCP=0x00;
	LDDR|=1<<LCD_D7|1<<LCD_D6|1<<LCD_D5|1<<LCD_D4;
	LCDR|=1<<LCD_E|1<<LCD_RW|1<<LCD_RS;
   //---------one------
	LDP=0<<LCD_D7|0<<LCD_D6|1<<LCD_D5|1<<LCD_D4; //4 bit mode
	LCP|=1<<LCD_E|0<<LCD_RW|0<<LCD_RS;		
	_delay_ms(1);
	LCP&=~(1<<LCD_E);
	_delay_ms(1);
	//-----------two-----------
	LDP=0<<LCD_D7|0<<LCD_D6|1<<LCD_D5|1<<LCD_D4; //4 bit mode
	LCP|=1<<LCD_E|0<<LCD_RW|0<<LCD_RS;		
	_delay_ms(1);
	LCP&=~(1<<LCD_E);
	_delay_ms(1);
	//-------three-------------
	LDP=0<<LCD_D7|0<<LCD_D6|1<<LCD_D5|0<<LCD_D4; //4 bit mode
	LCP|=1<<LCD_E|0<<LCD_RW|0<<LCD_RS;		
	_delay_ms(1);
	LCP&=~(1<<LCD_E);
	_delay_ms(1);
	//--------4 bit--dual line---------------
	LCD_sendCommand(0b00101000);
   //-----increment address, invisible cursor shift------
	LCD_sendCommand(0b00001100);
		//init 8 custom chars
	uint8_t ch=0, chn=0;
	while(ch<64)
	{
		LCD_defineChar((LcdCustomChar+ch),chn++);
		ch=ch+8;
	}


#else
	//8 bit part
	_delay_ms(15);
	LDP=0x00;
	LCP=0x00;
	LDDR|=1<<LCD_D7|1<<LCD_D6|1<<LCD_D5|1<<LCD_D4|1<<LCD_D3
			|1<<LCD_D2|1<<LCD_D1|1<<LCD_D0;
	LCDR|=1<<LCD_E|1<<LCD_RW|1<<LCD_RS;
   //---------one------
	LDP=0<<LCD_D7|0<<LCD_D6|1<<LCD_D5|1<<LCD_D4|0<<LCD_D3
			|0<<LCD_D2|0<<LCD_D1|0<<LCD_D0; //8 it mode
	LCP|=1<<LCD_E|0<<LCD_RW|0<<LCD_RS;		
	_delay_ms(1);
	LCP&=~(1<<LCD_E);
	_delay_ms(1);
	//-----------two-----------
	LDP=0<<LCD_D7|0<<LCD_D6|1<<LCD_D5|1<<LCD_D4|0<<LCD_D3
			|0<<LCD_D2|0<<LCD_D1|0<<LCD_D0; //8 it mode
	LCP|=1<<LCD_E|0<<LCD_RW|0<<LCD_RS;		
	_delay_ms(1);
	LCP&=~(1<<LCD_E);
	_delay_ms(1);
	//-------three-------------
	LDP=0<<LCD_D7|0<<LCD_D6|1<<LCD_D5|1<<LCD_D4|0<<LCD_D3
			|0<<LCD_D2|0<<LCD_D1|0<<LCD_D0; //8 it mode
	LCP|=1<<LCD_E|0<<LCD_RW|0<<LCD_RS;		
	_delay_ms(1);
	LCP&=~(1<<LCD_E);
	_delay_ms(1);
	//--------8 bit dual line----------
	LDP=0<<LCD_D7|0<<LCD_D6|1<<LCD_D5|1<<LCD_D4|1<<LCD_D3
			|0<<LCD_D2|0<<LCD_D1|0<<LCD_D0; //8 it mode
	LCP|=1<<LCD_E|0<<LCD_RW|0<<LCD_RS;		
	_delay_ms(1);
	LCP&=~(1<<LCD_E);
	_delay_ms(1);
   //-----increment address, invisible cursor shift------
	LDP=0<<LCD_D7|0<<LCD_D6|0<<LCD_D5|0<<LCD_D4|1<<LCD_D3
			|1<<LCD_D2|0<<LCD_D1|0<<LCD_D0; //8 it mode
	LCP|=1<<LCD_E|0<<LCD_RW|0<<LCD_RS;		
	_delay_ms(1);
	LCP&=~(1<<LCD_E);
	_delay_ms(5);
		//init custom chars
	uint8_t ch=0, chn=0;
	while(ch<64)
	{
		LCD_defineChar((LcdCustomChar+ch),chn++);
		ch=ch+8;
	}

#endif
}


/************************************************************************/
/* @clear lcd                                                                     */
/************************************************************************/			
void LCD_clr(void){
	
	LCD_sendCommand(1<<LCD_CLR);
	_delay_us(650);
}

/************************************************************************/
/* @set cursor home                                                                     */
/************************************************************************/
void LCD_home(void){
	
	LCD_sendCommand(1<<LCD_HOME);
}


/************************************************************************/
/* @send string CHEC, change to end of                                                                     */
/************************************************************************/
void LCD_sendString(char *data){

register uint8_t i;
uint8_t len=0;
	len = strlen(data);

	// check to make sure we have a good pointer
	if (!data) return;

	for (i=0; i<len;i++)
		LCD_sendChar(data[i]);

/*	// print data
	for(i=0; i<nBytes; i++)
		LCD_sendChar(data[i]);
*/
}


/************************************************************************/
/* @set lcd position                                                                     */
/************************************************************************/
void LCD_gotoXY(uint8_t x, uint8_t y)
{
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
	LCD_sendCommand(1<<LCD_DDRAM | DDRAMAddr);
	
}


//Copies string from flash memory to LCD at x y position
//const uint8_t welcomeln1[] PROGMEM="AVR LCD DEMO\0";
//CopyStringtoLCD(welcomeln1, 3, 1);	
void LCD_copyStringtoLCD(const uint8_t *FlashLoc, uint8_t x, uint8_t y)
{
	uint8_t i;
	LCD_gotoXY(x,y);
	for(i=0;(uint8_t)pgm_read_byte(&FlashLoc[i]);i++)
	{
		LCD_sendChar((uint8_t)pgm_read_byte(&FlashLoc[i]));
	}
}
//defines char symbol in CGRAM
/*
const uint8_t backslash[] PROGMEM= 
{
0b00000000,//back slash
0b00010000,
0b00001000,
0b00000100,
0b00000010,
0b00000001,
0b00000000,
0b00000000
};
LCDdefinechar(backslash,0);
*/


/************************************************************************/
/* @define char                                                                     */
/************************************************************************/
void LCD_defineChar(const uint8_t *pc,uint8_t char_code){
	uint8_t a, pcc;
	uint16_t i;
	a=(char_code<<3)|0x40;
	for (i=0; i<8; i++){
		pcc=pgm_read_byte(&pc[i]);
		LCD_sendCommand(a++);
		LCD_sendChar(pcc);
		}
}


/************************************************************************/
/* @shift cursor  n caracter left                                                                     */
/************************************************************************/
void LCD_shiftLeft(uint8_t n){
	
	for (uint8_t i=0;i<n;i++)
		LCD_sendCommand(0x1E);
}

/************************************************************************/
/* @shift cursor  n caracter right                                                                    */
/************************************************************************/
void LCD_shiftRight(uint8_t n){
	for (uint8_t i=0;i<n;i++)
		LCD_sendCommand(0x18);
}


/************************************************************************/
/* @dyplay on cursor                                                                     */
/************************************************************************/
void LCD_setCursorOn(void){
	
	LCD_sendCommand(0x0E);
}


/************************************************************************/
/* @set cursot to blink                                                                     */
/************************************************************************/
void LCD_setCursorOnBlink(void){
	
	LCD_sendCommand(0x0F);
}


/************************************************************************/
/* @turn cursor off                                                                     */
/************************************************************************/
void LCD_setCursorOFF(void){
	
	LCD_sendCommand(0x0C);
}


/************************************************************************/
/* @set lcd blank                                                                     */
/************************************************************************/
void LCD_setBlank(void){
	LCD_sendCommand(0x08);
}


/************************************************************************/
/* @set lcd visible                                                                     */
/************************************************************************/
void LCD_setVisible(void){
	
	LCD_sendCommand(0x0C);
}


/************************************************************************/
/* @move cursor n positions letf                                                                     */
/************************************************************************/
void LCD_cursorLeft(uint8_t n){
	
	for (uint8_t i=0;i<n;i++)
		LCD_sendCommand(0x10);
}


/************************************************************************/
/* @move cursor n positions right                                                                     */
/************************************************************************/
void LCD_cursorRight(uint8_t n){
	
	for (uint8_t i=0;i<n;i++)
		LCD_sendCommand(0x14);
}



/************************************************************************/
/* @show progress bar annime                                                                   */
/************************************************************************/
void LCD_progressBar(uint8_t progress, uint8_t maxprogress, uint8_t length){
	uint8_t i;
	uint16_t pixelprogress;
	uint8_t c;

	// draw a progress bar displaying (progress / maxprogress)
	// starting from the current cursor position
	// with a total length of "length" characters
	// ***note, LCD chars 0-5 must be programmed as the bar characters
	// char 0 = empty ... char 5 = full

	// total pixel length of bargraph equals length*PROGRESSPIXELS_PER_CHAR;
	// pixel length of bar itself is
	pixelprogress = ((progress*(length*PROGRESSPIXELS_PER_CHAR))/maxprogress);
	
	// print exactly "length" characters
	for(i=0; i<length; i++)
	{
		// check if this is a full block, or partial or empty
		// (u16) cast is needed to avoid sign comparison warning
		if( ((i*(uint16_t)PROGRESSPIXELS_PER_CHAR)+5) > pixelprogress )
		{
			// this is a partial or empty block
			if( ((i*(uint16_t)PROGRESSPIXELS_PER_CHAR)) > pixelprogress )
			{
				// this is an empty block
				// use space character?
				c = 0;
			}
			else
			{
				// this is a partial block
				c = pixelprogress % PROGRESSPIXELS_PER_CHAR;
			}
		}
		else
		{
			// this is a full block
			c = 5;
		}
		
		// write character to display
		LCD_sendChar(c);
	}

}
