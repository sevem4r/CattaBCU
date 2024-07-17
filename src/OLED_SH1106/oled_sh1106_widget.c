/*
 * oled_sh1106_widget.c
 *
 *  Created on: 15.11.2022
 *      Author: mariusz
 */

#include <inttypes.h>
#include <string.h>
#include "oled_sh1106.h"
#include "oled_sh1106_widget.h"

enum WIDGET_TYPE{
	WIDGET_TYPE_IMG,
	WIDGET_TYPE_LABEL
};

static Size_t oled_sh1106_get_text_size(char* text, Font_t* font){
	uint8_t len = strlen(text);
	Size_t size;
	uint32_t offset;

	size.width = 0;
	size.height = 0;

	for(uint32_t s=0; s<len; s++){
		offset = font->offset[text[s] - font->start];
		size.width += font->bitmap[offset];
		size.height += font->bitmap[offset + 1];
	}

	if(len){
		size.height = size.height / len;
	}

	return size;
}

static Size_t oled_sh1106_calculate_dsize(Widget_t* widget){
	Size_t dsize;

	dsize.width = 0;
	dsize.height = 0;

	// width
	if(widget->wmode == LCD_WMODE_LEFT){
		dsize.width = 0;
	}
	else if(widget->wmode == LCD_WMODE_RIGHT){
		dsize.width = -widget->size.width;
	}
	else if(widget->wmode == LCD_WMODE_CENTER){
		if(widget->size.width != 0){
			dsize.width = -widget->size.width / 2;
		}
	}


	// height
	if(widget->hmode == LCD_HMODE_UP){
		dsize.height = 0;
	}
	else if(widget->hmode == LCD_HMODE_DOWN){
		dsize.height = widget->size.height;
	}
	else if(widget->hmode == LCD_HMODE_CENTER){
		if(dsize.height != 0){
			dsize.height = -dsize.height / 2;
		}
	}

	return dsize;
}

void oled_sh1106_text_widget(
	Widget_t* widget,
	int16_t x,
	int16_t y,
	char* text,
	Font_t* font,
	uint32_t wmode,
	uint32_t hmode,
	Widget_t* parent)
{
	widget->label.font = font;
	widget->label.text = text;

	widget->wmode = wmode;
	widget->hmode = hmode;

	widget->position_requested.x = x;
	widget->position_requested.y = y;

	widget->size = oled_sh1106_get_text_size(widget->label.text, widget->label.font);

	widget->type = WIDGET_TYPE_LABEL;

	widget->parent = parent;

	if(parent){
		Widget_t* child;

		if(parent->child){
			child = parent->child;
			while(child->child){
				child = child->child;
			}

			child->child = widget;
		}
		else{
			parent->child = widget;
		}
	}

	widget->child = 0;
}

void oled_sh1106_draw_widget(
	Widget_t* widget)
{
	while(widget){
		switch(widget->type){
		case WIDGET_TYPE_IMG:
//			widget->size = LCD_get_img_size(widget->img);
//
//			if(widget->parent){
//				Size_t dsize;
//
//				dsize = LCD_calculate_dsize(widget);
//
//				x = widget->parent->x + dsize.width;
//				y = widget->parent->y + dsize.height;
//			}
//
//			if(widget->compresion){
//				if(widget->pbar.enable){
//					LCD_draw_pbar_rle(widget->img, widget->pbar.value, x, y, addr, widget->modulate);
//				}
//				else{
//					LCD_draw_image_rle(widget->img, x, y, addr, widget->modulate);
//				}
//			}
//			else{
//				if(widget->pbar.enable){
//					LCD_draw_image(widget->img, x, y, addr, widget->modulate);
//				}
//				else{
//					LCD_draw_image(widget->img, x, y, addr, widget->modulate);
//				}
//			}
			break;

		case WIDGET_TYPE_LABEL:
			if(widget->parent){
				Size_t dsize;

				dsize = oled_sh1106_calculate_dsize(widget);

				widget->position_top_left.x = widget->parent->position_top_left.x + widget->position_requested.x + dsize.width;
				widget->position_top_left.y = widget->parent->position_top_left.y + widget->position_requested.y + dsize.height;
			}
			else{
				Size_t dsize;

				dsize = oled_sh1106_calculate_dsize(widget);

				widget->position_top_left.x = widget->position_requested.x + dsize.width;
				widget->position_top_left.y = widget->position_requested.y + dsize.height;
			}

			oled_sh1106_draw_text(widget->label.font, widget->position_top_left.x, widget->position_top_left.y, widget->label.text);

			break;
		}

		widget = widget->child;
	}
}
