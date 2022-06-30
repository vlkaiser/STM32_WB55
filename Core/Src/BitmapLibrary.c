/*
 * JaktoolLogo.c
 *
 *  Created on: May 28, 2021
 *      Author: vkaiser
 */
#include "BitmapLibrary.h"
#include "ssd1306.h"
#include <math.h>


/**************************************************************************/
/*!
    @brief      Uses polyline (ssd1306.c) to draw lines based on coordinates in struct
    @param
*/
/**************************************************************************/
void ssd1306_drawLightningArrow()
{
    ssd1306_Fill(Black);	//Clear the screen

	  SSD1306_VERTEX loc_vertex[] =
	  {
	      {35,40},
	      {40,20},
	      {45,28},
	      {50,10},
	      {45,16},
	      {50,10},
	      {53,16}
	  };
	  ssd1306_Polyline(loc_vertex,sizeof(loc_vertex)/sizeof(loc_vertex[0]),White);
	  ssd1306_UpdateScreen();
	  return;
}

/**************************************************************************/
/*!
   @brief      Draw a RAM-resident 1-bit image at the specified (x,y) position,
   using the specified foreground (for set bits) and background (unset bits)
   colors.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with monochrome bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
    @param    color 16-bit 5-6-5 Color to draw pixels with

    @note	https://diyusthad.com/image2cpp:
    		assumes BLACK Background, INVERT Image colors, HORIZONTAL Draw Mode
*/
/**************************************************************************/

void drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w,
        int16_t h, uint16_t color, uint16_t bg)
{

    ssd1306_Fill(bg);		//Clear the screen

	int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
	uint8_t byte = 0;

	//Height
	for (int16_t j = 0; j < h; j++, y++)
	{
		//Width
		for (int16_t i = 0; i < w; i++)
		{
				byte = bitmap[j * byteWidth + i / 8];		// Row/Col of byte

				for (int8_t cnt = 0; cnt < 8; cnt ++)
				{
					//Shift left to right through byte to get each bit if the return val > 0, there is a 1 in that space.
					if ( (byte & (0x80 >> cnt)) == 0)
					{
						//  If 0, Background
						ssd1306_DrawPixel( x + i + cnt, y, bg);
					}
					else
					{
						//if not 0, Pixel Color
						ssd1306_DrawPixel( x + i + cnt, y, color );
					}
				}
				// Increment x-pos by 1 byte
				i = i + 7;
			}
		}

	  ssd1306_UpdateScreen();
	  return;

}





