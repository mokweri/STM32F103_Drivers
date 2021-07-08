/*
 * lcd.c
 *
 *  Created on: 8 Jul 2021
 *      Author: OBED
 */

#include <stdio.h>

#include "lcd.h"

static void send(uint8_t, uint8_t);
static void write4bits(uint8_t);
static void write8bits(uint8_t);
static void pulseEnable();
static void Pin_Init(void);
static void _delay_ms(uint32_t cnt);
static void _delay_us(uint32_t cnt);

uint8_t _data_pins[8];

uint8_t _displayfunction;
uint8_t _displaycontrol;
uint8_t _displaymode;

uint8_t _initialized;

uint8_t _numlines;
uint8_t _row_offsets[4];

void _delay_ms(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1000); i++);
}
static void _delay_us(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1); i++);
}

void Pin_Init(void)
{
	GPIO_Handle_t lcd;

	lcd.pGPIOx = LCD_PORT;
	lcd.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_PP;
	lcd.GPIO_PinConfig.GPIO_PinNumber = LCD_RS;
	GPIO_Init(&lcd);

	lcd.GPIO_PinConfig.GPIO_PinNumber = LCD_RW;
	GPIO_Init(&lcd);

	lcd.GPIO_PinConfig.GPIO_PinNumber = LCD_EN;
	GPIO_Init(&lcd);

	// Do these once, instead of every time a character is drawn for speed reasons.
	for (int i=0; i<((_displayfunction & LCD_8BITMODE) ? 8 : 4); ++i)
	{
		lcd.GPIO_PinConfig.GPIO_PinNumber = _data_pins[i];
		GPIO_Init(&lcd);
	}

}

void lcd_init(void)
{
	_data_pins[0] = LCD_D4;
	_data_pins[1] = LCD_D5;
	_data_pins[2] = LCD_D6;
	_data_pins[3] = LCD_D7;

	_data_pins[4] = LCD_D3;
	_data_pins[5] = LCD_D2;
	_data_pins[6] = LCD_D1;
	_data_pins[7] = LCD_D0;

	//4-bit mode
	_displayfunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
	//8-bit mode
	//_displayfunction = LCD_8BITMODE | LCD_1LINE | LCD_5x8DOTS;

	_numlines = 2;

	//pin initialiazation
	Pin_Init();

	// SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
	// according to datasheet, we need at least 40 ms after power rises above 2.7 V
	// before sending commands. we'll wait 50
	_delay_ms(40);

	// Now we pull both RS and R/W low to begin commands
	GPIO_WriteToOutputPin(LCD_PORT, LCD_RS, RESET);
	GPIO_WriteToOutputPin(LCD_PORT, LCD_EN, RESET);
	GPIO_WriteToOutputPin(LCD_PORT, LCD_RW, RESET);

	//put the LCD into 4 bit or 8 bit mode
	if (! (_displayfunction & LCD_8BITMODE)) {
		// this is according to the Hitachi HD44780 datasheet figure 24, pg 46

		// we start in 8bit mode, try to set 4 bit mode
		write4bits(0x03);
		_delay_us(4500); //wait min 4.1ms

		// second try
		write4bits(0x03);
		_delay_us(4500); //wait min 4.1ms

		// third go!
		write4bits(0x03);
		_delay_us(150);

		// finally, set to 4-bit interface
		write4bits(0x02);
	}else{
		// this is according to the Hitachi HD44780 datasheet page 45 figure 23

		// Send function set command sequence
		command(LCD_FUNCTIONSET | _displayfunction);
		_delay_ms(5); // wait more than 4.1 ms

		// second try
		command(LCD_FUNCTIONSET | _displayfunction);
		_delay_us(150);

		// third go
		command(LCD_FUNCTIONSET | _displayfunction);
	}

	// finally, set # lines, font size, etc.
	command(LCD_FUNCTIONSET | _displayfunction);

	// turn the display on with no cursor or blinking default
	_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	lcd_display_on();

	// clear it off
	lcd_clear();

	// Initialize to default text direction (for romance languages)
	_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
	// set the entry mode
	command(LCD_ENTRYMODESET | _displaymode);

}

void lcd_setRowOffsets(int row1, int row2, int row3, int row4)
{
	_row_offsets[0] = row1;
	_row_offsets[1] = row2;
	_row_offsets[2] = row3;
	_row_offsets[3] = row4;
}

void lcd_clear()
{
	command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
	_delay_us(2000); // this command takes a long time!
}

void lcd_home()
{
	command(LCD_RETURNHOME);  // set cursor position to zero
	_delay_us(2000);  // this command takes a long time!
}

void lcd_setCursor(uint8_t col, uint8_t row)
{
	lcd_setRowOffsets(0x00, 0x40, 0x14, 0x54);

	const size_t max_lines = sizeof(_row_offsets) / sizeof(*_row_offsets);
	if ( row >= max_lines ) {
		row = max_lines - 1;    // we count rows starting w/ 0
	}
	if ( row >= _numlines ) {
		row = _numlines - 1;    // we count rows starting w/ 0
	}

	command(LCD_SETDDRAMADDR | (col + _row_offsets[row]));

}

// Turn the display on/off (quickly)
void lcd_display_on()
{
	_displaycontrol |= LCD_DISPLAYON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_display_off()
{
	_displaycontrol &= ~LCD_DISPLAYON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void lcd_noCursor()
{
	_displaycontrol &= ~LCD_CURSORON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_cursor()
{
	_displaycontrol |= LCD_CURSORON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void lcd_noBlink()
{
	_displaycontrol &= ~LCD_BLINKON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_blink()
{
	_displaycontrol |= LCD_BLINKON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void lcd_scrollDisplayLeft()
{
	command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void lcd_scrollDisplayRight()
{
	command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void lcd_leftToRight()
{
	_displaymode |= LCD_ENTRYLEFT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void lcd_rightToLeft()
{
	_displaymode &= ~LCD_ENTRYLEFT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'right justify' text from the cursor
void lcd_autoscroll()
{
	_displaymode |= LCD_ENTRYSHIFTINCREMENT;
	command(LCD_ENTRYMODESET | _displaymode);
}
// This will 'left justify' text from the cursor
void lcd_noAutoscroll()
{
	_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void lcd_createChar(uint8_t location, uint8_t charmap[])
{
	  location &= 0x7; // we only have 8 locations 0-7
	  command(LCD_SETCGRAMADDR | (location << 3));
	  for (int i=0; i<8; i++) {
		  write(charmap[i]);
	  }
}

/*********** mid level commands, for sending data/cmds */
void command(uint8_t cmd)
{
	send(cmd, 0);
}

uint8_t write(uint8_t data)
{
	send(data, 1);
	return 1; // assume success
}

/************ low level data pushing commands **********/
void send(uint8_t value, uint8_t mode)
{
	if (mode){GPIO_WriteToOutputPin(LCD_PORT, LCD_RS, SET);}else{
		GPIO_WriteToOutputPin(LCD_PORT, LCD_RS, RESET);}

	// if there is a RW pin indicated, set it low to Write
	GPIO_WriteToOutputPin(LCD_PORT, LCD_RW, RESET);

	if (_displayfunction & LCD_8BITMODE)
	{
		write8bits(value);
	} else {
		write4bits(value>>4);
		write4bits(value);
	}
}

void pulseEnable(void)
{
	GPIO_WriteToOutputPin(LCD_PORT, LCD_EN, RESET);
	_delay_us(1);
	GPIO_WriteToOutputPin(LCD_PORT, LCD_EN, SET);
	_delay_us(1);			// enable pulse must be >450 ns
	GPIO_WriteToOutputPin(LCD_PORT, LCD_EN, RESET);
	_delay_us(100);			// commands need >37 us to settle

}

void write4bits(uint8_t value)
{
	for(int i=0; i<4; i++)
	{
		if((value >> i) & 0x01)
		{
			GPIO_WriteToOutputPin(LCD_PORT, _data_pins[i],SET);
		}else{
			GPIO_WriteToOutputPin(LCD_PORT, _data_pins[i],RESET);
		}
	}
	pulseEnable();
}

void write8bits(uint8_t value)
{
	for(int i=0; i<8; i++)
	{
		if((value >>1 ) & 0x01){ GPIO_WriteToOutputPin(LCD_PORT, _data_pins[i],SET); }else
		{ GPIO_WriteToOutputPin(LCD_PORT, _data_pins[i],RESET); }
	}
	pulseEnable();
}

//------Print------------
void lcd_puts(char *string) {
	for (char *it = string; *it; it++) {
		write(*it);
	}
}

void LCD_PrintString(char *string, uint8_t x, uint8_t y)
{
	lcd_setCursor(x,y);
	lcd_puts(string);
}
