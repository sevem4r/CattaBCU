/*
 * lights.c
 *
 *  Created on: 14.11.2022
 *      Author: mariusz
 */

#include "images.h"

const uint8_t lights_bitmap[] = {
	0xAA, 0xAA, 0x02, 0xF8, 0x8E, 0x02, 0x02, 0x06, 0x04, 0x8C, 0xF8, 0x02, 0x02, 0x02, 0x00, 0x03,
	0x02, 0x02, 0x03, 0x01, 0x01, 0x00,
};

Image_t lights = {
	.width = 11,
	.height = 11,
	.bitmap = lights_bitmap
};
