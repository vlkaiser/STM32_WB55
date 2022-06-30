/*
 * OLED.c
 *
 *  Created on: May 28, 2021
 *      Author: vkaiser
 *
 *  REQUIRES STM32-SSD1306.h/.c LIBRARIES
 *  - SSD1306.c/.h
 *  - SSD1306_fonts.c/.h
 *  - SSD1306_conf.h
 */


#include <string.h>
#include <stdio.h>
#include "ssd1306.h"
#include "OLED.h"

/**************************************************************************/
/*!
   @brief     Write in 16x26 font
    @param    x   		Top left corner x coordinate
    @param    y   		Top left corner y coordinate
    @param    color		Pixel Color (Black / White)
    @param	  str	 	Text in quotes to print to the screen eg "Initializing..."

    @note	If you don't want overlapping text, clear the screen.
*/
/**************************************************************************/
void writeLargeFont(int16_t x, int16_t y, uint16_t color, char* str)
{
	ssd1306_SetCursor(x, y);
	ssd1306_WriteString(str, Font_16x26, color);

	ssd1306_UpdateScreen();
	return;
}

/**************************************************************************/
/*!
   @brief     Write in 11x18 font
    @param    x   		Top left corner x coordinate
    @param    y   		Top left corner y coordinate
    @param    color		Pixel Color (Black / White)
    @param	  str	 	Text in quotes to print to the screen eg "Initializing..."

    @note	If you don't want overlapping text, clear the screen.
*/
/**************************************************************************/
void writeMedFont(int16_t x, int16_t y, uint16_t color, char* str)
{
	ssd1306_SetCursor(x, y);
	ssd1306_WriteString(str, Font_11x18, color);

	ssd1306_UpdateScreen();
	return;
}

/**************************************************************************/
/*!
   @brief     Write in 7x10 font
    @param    x   		Top left corner x coordinate
    @param    y   		Top left corner y coordinate
    @param    color		Pixel Color (Black / White)
    @param	  str	 	Text in quotes to print to the screen eg "Initializing..."

    @note	If you don't want overlapping text, clear the screen.
*/
/**************************************************************************/
void writeSmFont(int16_t x, int16_t y, uint16_t color, char* str)
{
	ssd1306_SetCursor(x, y);
	ssd1306_WriteString(str, Font_7x10, color);

	ssd1306_UpdateScreen();
	return;
}

/**************************************************************************/
/*!
   @brief     Write in 6x8 font
    @param    x   		Top left corner x coordinate
    @param    y   		Top left corner y coordinate
    @param    color		Pixel Color (Black / White)
    @param	  str	 	Text in quotes to print to the screen eg "Initializing..."

    @note	If you don't want overlapping text, clear the screen.
*/
/**************************************************************************/
void writeTinyFont(int16_t x, int16_t y, uint16_t color, char* str)
{
	ssd1306_SetCursor(x, y);
	ssd1306_WriteString(str, Font_6x8, color);

	ssd1306_UpdateScreen();
	return;
}

void clearScreen(uint16_t color)
{
	ssd1306_Fill(color);
}

