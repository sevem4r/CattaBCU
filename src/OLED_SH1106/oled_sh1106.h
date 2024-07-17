/*
 * oled_sh1106.h
 *
 *  Created on: 28 wrz 2019
 *      Author: Mariusz
 */

#ifndef OLED_SH1106_OLED_SH1106_H_
#define OLED_SH1106_OLED_SH1106_H_

#define LCD_WIDTH	128
#define LCD_HEIGHT	64

typedef struct{
	char start;
	char end;
	const uint8_t* bitmap;
	const uint16_t* offset;
}Font_t;

typedef struct{
	uint8_t width;
	uint8_t height;
	const uint8_t* bitmap;
}Image_t;

void oled_sh1106_init(void);
void oled_sh1106_clear(void);
void oled_sh1106_display(void);
void oled_sh1106_draw_pixel(int16_t x, int16_t y, uint8_t color);
void oled_sh1106_draw_bitmap(Image_t* img, int16_t x, int16_t y);
void oled_sh1106_draw_text(Font_t* font, int16_t x, int16_t y, const char* text);

#endif /* OLED_SH1106_OLED_SH1106_H_ */
