#ifndef LCD_LIB
#define LCD_LIB

#include <inttypes.h>


//Uncomment this if LCD 4 bit interface is used
//******************************************
#define LCD_4bit
//***********************************************

#define LCD_RS	0 	//define MCU pin connected to LCD RS
#define LCD_RW	1 	//define MCU pin connected to LCD R/W
#define LCD_E	2	//define MCU pin connected to LCD E
#define LCD_D0	0	//define MCU pin connected to LCD D0
#define LCD_D1	1	//define MCU pin connected to LCD D1
#define LCD_D2	2	//define MCU pin connected to LCD D2
#define LCD_D3	3	//define MCU pin connected to LCD D3
#define LCD_D4	4	//define MCU pin connected to LCD D4
#define LCD_D5	5	//define MCU pin connected to LCD D5
#define LCD_D6	6	//define MCU pin connected to LCD D6
#define LCD_D7	7	//define MCU pin connected to LCD D7
#define LDP PORTC	//define MCU port connected to LCD data pins
#define LCP PORTC	//define MCU port connected to LCD control pins
#define LDDR DDRC	//define MCU direction register for port connected to LCD data pins
#define LCDR DDRC	//define MCU direction register for port connected to LCD control pins

#define LCD_CLR             0	//DB0: clear display
#define LCD_HOME            1	//DB1: return to home position
#define LCD_ENTRY_MODE      2	//DB2: set entry mode
#define LCD_ENTRY_INC       1	//DB1: increment
#define LCD_ENTRY_SHIFT     0	//DB2: shift
#define LCD_ON_CTRL         3	//DB3: turn lcd/cursor on
#define LCD_ON_DISPLAY      2	//DB2: turn display on
#define LCD_ON_CURSOR       1	//DB1: turn cursor on
#define LCD_ON_BLINK        0	//DB0: blinking cursor
#define LCD_MOVE            4	//DB4: move cursor/display
#define LCD_MOVE_DISP       3	//DB3: move display (0-> move cursor)
#define LCD_MOVE_RIGHT      2	//DB2: move right (0-> left)
#define LCD_FUNCTION        5	//DB5: function set
#define LCD_FUNCTION_8BIT   4	//DB4: set 8BIT mode (0->4BIT mode)
#define LCD_FUNCTION_2LINES 3	//DB3: two lines (0->one line)
#define LCD_FUNCTION_10DOTS 2	//DB2: 5x10 font (0->5x7 font)
#define LCD_CGRAM           6	//DB6: set CG RAM address
#define LCD_DDRAM           7	//DB7: set DD RAM address
// reading:
#define LCD_BUSY            7	//DB7: LCD is busy
#define LCD_LINES			4	//visible lines
#define LCD_LINE_LENGTH		20	//line length (in characters)
// cursor position to DDRAM mapping
#define LCD_LINE0_DDRAMADDR		0x00
#define LCD_LINE1_DDRAMADDR		0x40
#define LCD_LINE2_DDRAMADDR		0x14
#define LCD_LINE3_DDRAMADDR		0x54
// progress bar defines
#define PROGRESSPIXELS_PER_CHAR	6

//lcd struct
typedef struct lcd_t{
	//
	uint8_t nLines;
	uint8_t lenght;
	char line0Data[LCD_LINE_LENGTH+1];
	char line1Data[LCD_LINE_LENGTH+1];
	char line2Data[LCD_LINE_LENGTH+1];
	char line3Data[LCD_LINE_LENGTH+1];
	
	uint8_t flagUpdateLCD;
}lcd_t;
	
extern void LCD_sendChar(uint8_t);		//forms data ready to send to 74HC164
extern void LCD_sendCommand(uint8_t);	//forms data ready to send to 74HC164
extern void LCD_init(void);			//Initializes LCD
extern void LCD_clr(void);				//Clears LCD
extern void LCD_home(void);			//LCD cursor home
extern void LCD_sendString(char *data);	//Outputs string to LCD
extern void LCD_gotoXY(uint8_t, uint8_t);	//Cursor to X Y position
extern void LCD_copyStringtoLCD(const uint8_t *FlashLoc, uint8_t x, uint8_t y);//copies flash string to LCD at x,y
extern void LCD_defineChar(const uint8_t *,uint8_t);//write char to LCD CGRAM 
extern void LCD_shiftRight(uint8_t);	//shift by n characters Right
extern void LCD_shiftLeft(uint8_t);	//shift by n characters Left
extern void LCD_setCursorOn(void);		//Underline cursor ON
extern void LCD_setCursorOnBlink(void);	//Underline blinking cursor ON
extern void LCD_setCursorOFF(void);		//Cursor OFF
extern void LCD_setBlank(void);			//LCD blank but not cleared
extern void LCD_setVisible(void);			//LCD visible
extern void LCD_cursorLeft(uint8_t);	//Shift cursor left by n
extern void LCD_cursorRight(uint8_t);	//shif cursor right by n
// displays a horizontal progress bar at the current cursor location
// <progress> is the value the bargraph should indicate
// <maxprogress> is the value at the end of the bargraph
// <length> is the number of LCD characters that the bargraph should cover
//adapted from AVRLIB - displays progress only for 8 bit variables
extern void LCD_progressBar(uint8_t progress, uint8_t maxprogress, uint8_t length);


#endif

