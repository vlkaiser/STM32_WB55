/*
 * OLED.h
 *
 *  Created on: May 28, 2021
 *      Author: vkaiser
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_

#include <_ansi.h>

_BEGIN_STD_C


void writeLargeFont(int16_t x, int16_t y, uint16_t color, char* str);
void writeMedFont(int16_t x, int16_t y, uint16_t color, char* str);
void writeSmFont(int16_t x, int16_t y, uint16_t color, char* str);
void writeTinyFont(int16_t x, int16_t y, uint16_t color, char* str);
void clearScreen(uint16_t color);

_END_STD_C
#endif /* INC_OLED_H_ */
