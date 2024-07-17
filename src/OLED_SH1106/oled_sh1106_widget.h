/*
 * oled_sh1106_widget.h
 *
 *  Created on: 15.11.2022
 *      Author: mariusz
 */

#ifndef OLED_SH1106_OLED_SH1106_WIDGET_H_
#define OLED_SH1106_OLED_SH1106_WIDGET_H_

typedef struct Widget_s Widget_t;

typedef struct{
	int16_t x;
	int16_t y;
}Position_t;

typedef struct{
	int16_t width;
	int16_t height;
}Size_t;

typedef struct{
	Font_t* font;
	char* text;
}Label_t;

struct Widget_s{
	union{
		Label_t label;
		Image_t* img;
	};

	uint32_t wmode;
	uint32_t hmode;

	Position_t position_requested;
	Position_t position_top_left;

	Size_t size;

	uint32_t type;

	Widget_t* parent;
	Widget_t* child;
};

enum LCD_WMODE{
	LCD_WMODE_LEFT,
	LCD_WMODE_RIGHT,
	LCD_WMODE_CENTER
};

enum LCD_HMODE{
	LCD_HMODE_UP,
	LCD_HMODE_DOWN,
	LCD_HMODE_CENTER,
};

void oled_sh1106_text_widget(Widget_t* widget, int16_t x, int16_t y, char* text, Font_t* font, uint32_t wmode, uint32_t hmode, Widget_t* parent);
void oled_sh1106_draw_widget(Widget_t* widget);

#endif /* OLED_SH1106_OLED_SH1106_WIDGET_H_ */
