/*
 * temp.c
 *
 *  Created on: 14.11.2022
 *      Author: mariusz
 */

#include "images.h"

const uint8_t temp_bitmap[] = {
	0x00, 0x00, 0x00, 0x00, 0xFE, 0x01, 0xC1, 0x01, 0xFE, 0x00, 0xAA, 0xAA, 0x22, 0xC0, 0x30, 0x88,
	0xE8, 0xA7, 0xF0, 0x7F, 0xB0, 0xA7, 0xE8, 0x8A, 0x32, 0xC2, 0x03, 0x0C, 0x11, 0x17, 0x27, 0x2F,
	0x2E, 0x2D, 0x25, 0x17, 0x11, 0x0C, 0x03,
};

Image_t temp = {
	.width = 13,
	.height = 22,
	.bitmap = temp_bitmap
};
