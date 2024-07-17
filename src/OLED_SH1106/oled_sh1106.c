/*
 * oled_sh1106.c
 *
 *  Created on: 28 wrz 2019
 *      Author: Mariusz
 */
#include <inttypes.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "oled_sh1106.h"
#include "oled_sh1106_conf.h"

#define SH1106_SETCONTRAST 0x81
#define SH1106_DISPLAYALLON_RESUME 0xA4
#define SH1106_DISPLAYALLON 0xA5
#define SH1106_NORMALDISPLAY 0xA6
#define SH1106_INVERTDISPLAY 0xA7
#define SH1106_DISPLAYOFF 0xAE
#define SH1106_DISPLAYON 0xAF

#define SH1106_SETDISPLAYOFFSET 0xD3
#define SH1106_SETCOMPINS 0xDA

#define SH1106_SETVCOMDETECT 0xDB

#define SH1106_SETDISPLAYCLOCKDIV 0xD5
#define SH1106_SETPRECHARGE 0xD9

#define SH1106_SETMULTIPLEX 0xA8

#define SH1106_SETLOWCOLUMN 0x02 //to use with SSD1306, set to 0x00
#define SH1106_SETHIGHCOLUMN 0x10

#define SH1106_SETSTARTLINE 0x40

#define SH1106_MEMORYMODE 0x20
#define SH1106_COLUMNADDR 0x21
#define SH1106_PAGEADDR   0x22

#define SH1106_COMSCANINC 0xC0
#define SH1106_COMSCANDEC 0xC8

#define SH1106_SEGREMAP 0xA0

#define SH1106_CHARGEPUMP 0x8D

#define SH1106_EXTERNALVCC 0x1
#define SH1106_SWITCHCAPVCC 0x2


static uint8_t oled_sh1106_buffer[1024];

static void oled_sh1106_command(uint8_t cmd){
	_oled_sh1106_dc_clr();
	_oled_sh1106_cs_clr();
	_oled_sh1106_spi_transfer(cmd);
	_oled_sh1106_cs_set();
}

void oled_sh1106_clear(void){
	memset(oled_sh1106_buffer, 0x00, 1024);
}

void oled_sh1106_display(void){
	for(uint8_t i = 0; i < 8; i++){
		oled_sh1106_command(0xB0 + i);				// Set row
		oled_sh1106_command(SH1106_SETLOWCOLUMN);	// Set lower column address
		oled_sh1106_command(SH1106_SETHIGHCOLUMN); 	// Set higher column address

		_oled_sh1106_cs_set();
		_oled_sh1106_dc_set();
		_oled_sh1106_cs_clr();

		for(uint8_t j = 0; j < 128; j++){
			_oled_sh1106_spi_transfer(oled_sh1106_buffer[i * 128 + j]);
		}

		_oled_sh1106_cs_set();
	}
}

void oled_sh1106_draw_pixel(int16_t x, int16_t y, uint8_t color){
	if((x < 0) || (x >= 128) || (y < 0) || (y >= 64)){
		return;
	}

	if(color){
		oled_sh1106_buffer[x + (y / 8) * 128] |= (1 << (y & 0x07));
	}
	else{
		oled_sh1106_buffer[x + (y / 8) * 128] &= ~(1 << (y & 0x07));
	}
}

static void oled_sh1106_draw(const uint8_t* bitmap, uint8_t w, uint8_t h, int16_t x, int16_t y){
	uint16_t index;
	int16_t pX;
	int16_t pY;
	uint8_t tmp_byte;
	uint8_t bL;

	index = 0;
	pY = y;
	while(pY < (y + h)){
		pX = x;
		while(pX < (x + w)){
			bL = 0;
			tmp_byte = bitmap[index++];
			if(tmp_byte){
				while(bL < 8){
					if(tmp_byte & 0x01){
						oled_sh1106_draw_pixel(pX, pY + bL, 1);
					}

					tmp_byte >>= 1;
					if(tmp_byte){
						bL++;
					}
					else{
						pX++;
						break;
					}
				}
			}
			else{
				pX++;
			}
		}
		pY += 8;
	}
}

void oled_sh1106_draw_bitmap(Image_t* img, int16_t x, int16_t y){
	oled_sh1106_draw(img->bitmap, img->width, img->height, x, y);
}

void oled_sh1106_draw_text(Font_t* font, int16_t x, int16_t y, const char* text){
	const uint8_t* src;
	uint8_t w;
	uint8_t h;
	uint8_t len = strlen(text);

	for(uint8_t i = 0; i < len; i++){
		src = font->bitmap + font->offset[text[i] - font->start];
		w = src[0];
		h = src[1];

		oled_sh1106_draw(&src[2], w, h, x, y);
		x += w;
	}
}

void oled_sh1106_init(void){
	_oled_sh1106_init();

	_oled_sh1106_cs_set();
	_oled_sh1106_res_set();
	_oled_sh1106_delay_ms(100);
	_oled_sh1106_res_clr();
	_oled_sh1106_delay_ms(10);
	_oled_sh1106_res_set();

	oled_sh1106_command(SH1106_DISPLAYOFF);                    // 0xAE
	oled_sh1106_command(SH1106_SETDISPLAYCLOCKDIV);            // 0xD5
	oled_sh1106_command(0x80);                                 // the suggested ratio 0x80
	oled_sh1106_command(SH1106_SETMULTIPLEX);                  // 0xA8
	oled_sh1106_command(0x3F);
	oled_sh1106_command(SH1106_SETDISPLAYOFFSET);              // 0xD3
	oled_sh1106_command(0x00);                                 // no offset
	oled_sh1106_command(SH1106_SETSTARTLINE | 0x0);            // line #0
	oled_sh1106_command(SH1106_CHARGEPUMP);                    // 0x8D
	oled_sh1106_command(SH1106_MEMORYMODE);                    // 0x20
	oled_sh1106_command(0x00);                                 // 0x0 act like ks0108
	oled_sh1106_command(SH1106_SEGREMAP);
	oled_sh1106_command(0xC0);
	oled_sh1106_command(SH1106_SETCOMPINS);                    // 0xDA
	oled_sh1106_command(0x12);
	oled_sh1106_command(SH1106_SETCONTRAST);                   // 0x81
	oled_sh1106_command(SH1106_SETPRECHARGE);                  // 0xd9
	oled_sh1106_command(SH1106_SETVCOMDETECT);                 // 0xDB
	oled_sh1106_command(0x40);
	oled_sh1106_command(SH1106_DISPLAYALLON_RESUME);           // 0xA4
	oled_sh1106_command(SH1106_NORMALDISPLAY);                 // 0xA6

	oled_sh1106_command(0xC8); // 0xC8 - scan direction (flip up/down)
//	oled_sh1106_command(0xA7); // 0xA6 - reverse display
	oled_sh1106_command(0xA1); // 0xA1 - flip left/right
//	oled_sh1106_command(0x40);

	oled_sh1106_command(SH1106_DISPLAYON);

	oled_sh1106_clear();
}

//static void oled_sh1106_command( uint8_t cmd ){
//	_oled_sh1106_dc_clr();
//	_oled_sh1106_cs_clr();
//	_oled_sh1106_spi_transfer( cmd );
//	_oled_sh1106_cs_set();
//}
//
//void oled_init( void ){
//
//	_oled_sh1106_init();
//
//	_oled_sh1106_res_set();
//	_oled_sh1106_delay_ms( 100 );
//	_oled_sh1106_res_clr();
//	_oled_sh1106_delay_ms( 10 );
//	_oled_sh1106_res_set();
//
//	oled_sh1106_command( SH1106_DISPLAYOFF );                    // 0xAE
//	oled_sh1106_command( SH1106_SETDISPLAYCLOCKDIV );            // 0xD5
//	oled_sh1106_command( 0x80 );                                 // the suggested ratio 0x80
//	oled_sh1106_command( SH1106_SETMULTIPLEX );                  // 0xA8
//	oled_sh1106_command( 0x3F );
//	oled_sh1106_command( SH1106_SETDISPLAYOFFSET );              // 0xD3
//	oled_sh1106_command( 0x00 );                                 // no offset
//	oled_sh1106_command( SH1106_SETSTARTLINE | 0x0 );            // line #0
//	oled_sh1106_command( SH1106_CHARGEPUMP );                    // 0x8D
//	oled_sh1106_command( SH1106_MEMORYMODE );                    // 0x20
//	oled_sh1106_command( 0x00 );                                 // 0x0 act like ks0108
//	oled_sh1106_command( SH1106_SEGREMAP );
//	oled_sh1106_command( 0xC0 );
//	oled_sh1106_command( SH1106_SETCOMPINS );                    // 0xDA
//	oled_sh1106_command( 0x12 );
//	oled_sh1106_command( SH1106_SETCONTRAST );                   // 0x81
//	oled_sh1106_command( SH1106_SETPRECHARGE );                  // 0xd9
//	oled_sh1106_command( SH1106_SETVCOMDETECT );                 // 0xDB
//	oled_sh1106_command( 0x40 );
//	oled_sh1106_command( SH1106_DISPLAYALLON_RESUME );           // 0xA4
//	oled_sh1106_command( SH1106_NORMALDISPLAY );                 // 0xA6
//
//	oled_sh1106_command( 0xC8 ); // 0xC8 - scan direction (flip up/down)
////	oled_sh1106_command( 0xA7 ); // 0xA6 - reverse display
//	oled_sh1106_command( 0xA1 ); // 0xA1 - flip left/right
////	oled_sh1106_command( 0x40 );
//
//	oled_sh1106_command( SH1106_DISPLAYON );
//
//	oled_clear();
//}
//
////#define STACK_SIZE 	34
////
////typedef struct{
////	const uint8_t * src;
////	uint8_t x;
////	uint8_t y;
////	uint16_t it;
////}Element_t;
////
////uint8_t * oled_get_stack_size(){
////	static uint8_t stack_size;
////	return &stack_size;
////}
////
////Element_t * oled_get_stack( uint8_t id ){
////	static Element_t stack[STACK_SIZE];
////	return id < STACK_SIZE ? &stack[id] : 0;
////}
////
////void oled_clear_stack( void ){
////	uint8_t * size = oled_get_stack_size();
////	*size = 0;
////}
////
////void oled_add_to_stack( const uint8_t * src, uint8_t x, uint8_t y ){
////	uint8_t * size = oled_get_stack_size();
////	Element_t * element = oled_get_stack( *size );
////
////	element->src = src;
////	element->x = x;
////	element->y = y;
////
////	(*size)++;
////
////	uart_print( 0, "size: %d\r\n", *size );
////}
////
////void oled_display_stack( void ){
////	Element_t * stack = oled_get_stack( 0 );
////	uint8_t stackIndex = *oled_get_stack_size();
////
////	for( uint8_t l=0; l<stackIndex; l++ )
////		stack[l].it = 0;
////
////	for( uint8_t page_x=0; page_x<128; page_x++ ){
////
////		uint64_t oled_line = 0x00;
////		for( uint8_t l=0; l<stackIndex; l++ ){
////
////			uint8_t w = pgm_read_byte( stack[l].src );
////			uint8_t h = pgm_read_byte( stack[l].src + 1 );
////
////			if( page_x >= stack[l].x && page_x <= (stack[l].x + w - 1) ){
////				uint64_t img_line = 0x00;
////				uint8_t h_bytes = 1 + ((h - 1) / 8);
////
////				for( uint8_t i=0; i<h_bytes; i++ )
////					img_line |= (uint64_t)pgm_read_byte( stack[l].src+2+i+stack[l].it ) << (i * 8);
////
////				if( stack[l].y > 0 ) img_line <<= stack[l].y;
////				if( stack[l].y < 0 ) img_line >>= abs( stack[l].y );
////				oled_line |= img_line;
////				stack[l].it += h_bytes;
////			}
////		}
////
////		for( uint8_t page_y=0; page_y<8; page_y++ ){
////
////			oled_sh1106_command( 0xB0 + page_y );			// Set row
////
////			oled_sh1106_command( page_x & 0x0f );			// Set lower column address
////			oled_sh1106_command( 0x10 | (page_x >> 4) ); 	// Set higher column address
////
////			_oled_sh1106_cs_set();
////			_oled_sh1106_dc_set();
////			_oled_sh1106_cs_clr();
////
////			_oled_sh1106_spi_transfer( (oled_line >> (page_y * 8)) & 0xff );
////
////			_oled_sh1106_cs_set();
////		}
////	}
////}
//
//
////void oled_display2( const uint8_t * src, uint8_t x, uint8_t y ){
////	uint8_t w = src[0];
////	uint8_t h = src[1];
////	uint8_t y_start = y / 8;
////	uint8_t y_synch = y % 8;
////	uint8_t h_ceil = 1 + ((h - 1) / 8);
////	uint8_t passes = y_synch ? h_ceil + 1 : h_ceil;
////	uint64_t pages;
////	uint8_t * page = (uint8_t*)&pages;
////
////	for( uint8_t pass=0; pass<passes; pass++ ){
////		oled_sh1106_command( 0xB0 + y_start + pass );		// Set page
////
////		oled_sh1106_command( ((x + SH1106_SETLOWCOLUMN) & 0x0F) );						// Set lower column address
////		oled_sh1106_command( SH1106_SETHIGHCOLUMN | ((x + SH1106_SETLOWCOLUMN) >> 4) ); // Set higher column address
////
////		_oled_sh1106_cs_set();
////		_oled_sh1106_dc_set();
////		_oled_sh1106_cs_clr();
////
////		for( uint8_t i=0; i<w; i++ ){
////			pages = 0;
////			for( uint8_t row=0; row<h_ceil; row++ ){
////				page[row] = src[2 + i + row * w];
////			}
////			pages = pages << y_synch;
////
////			_oled_sh1106_spi_transfer( page[pass] );
////		}
////
////		_oled_sh1106_cs_set();
////	}
////}
//
////void oled_clear( void ){
////	for( uint8_t i=0; i<8; i++ ){
////		oled_sh1106_command( 0xB0 + i );				// Set row
////		oled_sh1106_command( SH1106_SETLOWCOLUMN );		// Set lower column address
////		oled_sh1106_command( SH1106_SETHIGHCOLUMN ); 	// Set higher column address
////
////		_oled_sh1106_cs_set();
////		_oled_sh1106_dc_set();
////		_oled_sh1106_cs_clr();
////
////		for( uint16_t j=0; j<128; j++ ){
////			_oled_sh1106_spi_transfer( 0x00 );
////		}
////
////		_oled_sh1106_cs_set();
////	}
////}
//
//#define OLED_SIZE_X 			128
//#define OLED_SIZE_Y				64
//#define OLED_BUFFER_SIZE		(OLED_SIZE_X * OLED_SIZE_Y / 8)
//#define OLED_BUFFER_0_START		0
//#define OLED_BUFFER_1_START		OLED_BUFFER_SIZE
//
//static uint8_t oled_double_buffer[2 * OLED_BUFFER_SIZE];
//
//void oled_clear(void){
//	memset(&oled_double_buffer[OLED_BUFFER_0_START], 0x00, OLED_BUFFER_SIZE);
//}
//
//void oled_display(void){
//	for(uint8_t i = 0; i < 8; i++){
//		oled_sh1106_command(0xB0 + i);				// Set row
//		oled_sh1106_command(SH1106_SETLOWCOLUMN);	// Set lower column address
//		oled_sh1106_command(SH1106_SETHIGHCOLUMN); 	// Set higher column address
//
//		_oled_sh1106_cs_set();
//		_oled_sh1106_dc_set();
//		_oled_sh1106_cs_clr();
//
//		for(uint8_t j = 0; j < 128; j++){
//			_oled_sh1106_spi_transfer(oled_double_buffer[OLED_BUFFER_0_START + i * 128 + j]);
//		}
//
//		_oled_sh1106_cs_set();
//	}
//}
//
//void oled_draw(const uint8_t* src, uint8_t x, uint8_t y){
//
////	uint8_t w = src[0];
////	uint8_t h = src[1];
////	uint8_t y_start = y / 8;
////	uint8_t y_synch = y % 8;
////	uint8_t h_ceil = 1 + ((h - 1) / 8);
////	uint8_t passes = y_synch ? h_ceil + 1 : h_ceil;
////	uint64_t pages;
////	uint8_t* page = (uint8_t*)&pages;
////
////	for(uint8_t pass = 0; pass < passes; pass++){
////		for(uint8_t i = 0; i < w; i++){
////			pages = 0;
////			for(uint8_t row = 0; row < h_ceil; row++){
////				page[row] = src[2 + i + row * w];
////
////
////			}
////			pages = pages << y_synch;
////		}
////	}
//
//	uint8_t index;
//	uint8_t col_min = x;
//	uint8_t col_max = col_min + src[0];
//	uint8_t row_min = 1;
//	uint8_t row_max = 2;
//
//	if(col_max > 128){
//		col_max = 128;
//	}
//
//	for(uint8_t row = row_min; row < row_max; row++){
//		index = 0;
//		for(uint8_t col = col_min; col < col_max; col++){
//			oled_double_buffer[OLED_BUFFER_0_START + row * 128 + col] = src[2 + row * src[0] + index];
//			index++;
//		}
//	}
//}
