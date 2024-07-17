/*
 * battery.c
 *
 *  Created on: 29.11.2022
 *      Author: mariusz
 */

#include "images.h"

const uint8_t battery_0_bitmap[] = {
		0xF8, 0x04, 0x04, 0x04, 0x07, 0x07, 0x07, 0x07, 0x04, 0x04, 0x04, 0xF8, 0xFF, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x07, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
		0x08, 0x08, 0x08, 0x07,
};

const uint8_t battery_1_bitmap[] = {
		0xF8, 0x04, 0x04, 0x04, 0x07, 0x07, 0x07, 0x07, 0x04, 0x04, 0x04, 0xF8, 0xFF, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x07, 0x08, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A,
		0x0A, 0x0A, 0x08, 0x07,
};

const uint8_t battery_2_bitmap[] = {
		0xF8, 0x04, 0x04, 0x04, 0x07, 0x07, 0x07, 0x07, 0x04, 0x04, 0x04, 0xF8, 0xFF, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x07, 0x08, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
		0x0B, 0x0B, 0x08, 0x07,
};

const uint8_t battery_3_bitmap[] = {
		0xF8, 0x04, 0x04, 0x04, 0x07, 0x07, 0x07, 0x07, 0x04, 0x04, 0x04, 0xF8, 0xFF, 0x00, 0x80, 0x80,
		0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0xFF, 0x07, 0x08, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
		0x0B, 0x0B, 0x08, 0x07,
};

const uint8_t battery_4_bitmap[] = {
		0xF8, 0x04, 0x04, 0x04, 0x07, 0x07, 0x07, 0x07, 0x04, 0x04, 0x04, 0xF8, 0xFF, 0x00, 0xC0, 0xC0,
		0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x00, 0xFF, 0x07, 0x08, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
		0x0B, 0x0B, 0x08, 0x07,
};

const uint8_t battery_5_bitmap[] = {
		0xF8, 0x04, 0x04, 0x04, 0x07, 0x07, 0x07, 0x07, 0x04, 0x04, 0x04, 0xF8, 0xFF, 0x00, 0xE0, 0xE0,
		0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0x00, 0xFF, 0x07, 0x08, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
		0x0B, 0x0B, 0x08, 0x07,
};

const uint8_t battery_6_bitmap[] = {
		0xF8, 0x04, 0x04, 0x04, 0x07, 0x07, 0x07, 0x07, 0x04, 0x04, 0x04, 0xF8, 0xFF, 0x00, 0xF0, 0xF0,
		0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0x00, 0xFF, 0x07, 0x08, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
		0x0B, 0x0B, 0x08, 0x07,
};

const uint8_t battery_7_bitmap[] = {
		0xF8, 0x04, 0x04, 0x04, 0x07, 0x07, 0x07, 0x07, 0x04, 0x04, 0x04, 0xF8, 0xFF, 0x00, 0xF8, 0xF8,
		0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0x00, 0xFF, 0x07, 0x08, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
		0x0B, 0x0B, 0x08, 0x07,
};

const uint8_t battery_8_bitmap[] = {
		0xF8, 0x04, 0x04, 0x04, 0x07, 0x07, 0x07, 0x07, 0x04, 0x04, 0x04, 0xF8, 0xFF, 0x00, 0xFC, 0xFC,
		0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0x00, 0xFF, 0x07, 0x08, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
		0x0B, 0x0B, 0x08, 0x07,
};

const uint8_t battery_9_bitmap[] = {
		0xF8, 0x04, 0x04, 0x04, 0x07, 0x07, 0x07, 0x07, 0x04, 0x04, 0x04, 0xF8, 0xFF, 0x00, 0xFE, 0xFE,
		0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0x00, 0xFF, 0x07, 0x08, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
		0x0B, 0x0B, 0x08, 0x07,
};

const uint8_t battery_10_bitmap[] = {
		0xF8, 0x04, 0x04, 0x04, 0x07, 0x07, 0x07, 0x07, 0x04, 0x04, 0x04, 0xF8, 0xFF, 0x00, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0x07, 0x08, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
		0x0B, 0x0B, 0x08, 0x07,
};

const uint8_t battery_11_bitmap[] = {
		0xF8, 0x04, 0x84, 0x84, 0x87, 0x87, 0x87, 0x87, 0x84, 0x84, 0x04, 0xF8, 0xFF, 0x00, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0x07, 0x08, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
		0x0B, 0x0B, 0x08, 0x07,
};

const uint8_t battery_12_bitmap[] = {
		0xF8, 0x04, 0xC4, 0xC4, 0xC7, 0xC7, 0xC7, 0xC7, 0xC4, 0xC4, 0x04, 0xF8, 0xFF, 0x00, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0x07, 0x08, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
		0x0B, 0x0B, 0x08, 0x07,
};

const uint8_t battery_13_bitmap[] = {
		0xF8, 0x04, 0xE4, 0xE4, 0xE7, 0xE7, 0xE7, 0xE7, 0xE4, 0xE4, 0x04, 0xF8, 0xFF, 0x00, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0x07, 0x08, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
		0x0B, 0x0B, 0x08, 0x07,
};

const uint8_t battery_14_bitmap[] = {
		0xF8, 0x04, 0xF4, 0xF4, 0xF7, 0xF7, 0xF7, 0xF7, 0xF4, 0xF4, 0x04, 0xF8, 0xFF, 0x00, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0x07, 0x08, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
		0x0B, 0x0B, 0x08, 0x07,
};

Image_t battery_0 = {
	.width = 12,
	.height = 20,
	.bitmap = battery_0_bitmap
};

Image_t battery_1 = {
	.width = 12,
	.height = 20,
	.bitmap = battery_1_bitmap
};

Image_t battery_2 = {
	.width = 12,
	.height = 20,
	.bitmap = battery_2_bitmap
};

Image_t battery_3 = {
	.width = 12,
	.height = 20,
	.bitmap = battery_3_bitmap
};

Image_t battery_4 = {
	.width = 12,
	.height = 20,
	.bitmap = battery_4_bitmap
};

Image_t battery_5 = {
	.width = 12,
	.height = 20,
	.bitmap = battery_5_bitmap
};

Image_t battery_6 = {
	.width = 12,
	.height = 20,
	.bitmap = battery_6_bitmap
};

Image_t battery_7 = {
	.width = 12,
	.height = 20,
	.bitmap = battery_7_bitmap
};

Image_t battery_8 = {
	.width = 12,
	.height = 20,
	.bitmap = battery_8_bitmap
};

Image_t battery_9 = {
	.width = 12,
	.height = 20,
	.bitmap = battery_9_bitmap
};

Image_t battery_10 = {
	.width = 12,
	.height = 20,
	.bitmap = battery_10_bitmap
};

Image_t battery_11 = {
	.width = 12,
	.height = 20,
	.bitmap = battery_11_bitmap
};

Image_t battery_12 = {
	.width = 12,
	.height = 20,
	.bitmap = battery_12_bitmap
};

Image_t battery_13 = {
	.width = 12,
	.height = 20,
	.bitmap = battery_13_bitmap
};

Image_t battery_14 = {
	.width = 12,
	.height = 20,
	.bitmap = battery_14_bitmap
};

Image_t* battery[15] = {
	&battery_0,
	&battery_1,
	&battery_2,
	&battery_3,
	&battery_4,
	&battery_5,
	&battery_6,
	&battery_7,
	&battery_8,
	&battery_9,
	&battery_10,
	&battery_11,
	&battery_12,
	&battery_13,
	&battery_14
};

//const uint8_t battery_0_bitmap[] = {
//	0xF8, 0x04, 0xB4, 0xB4, 0xB7, 0xB7, 0xB7, 0xB7, 0xB4, 0xB4, 0x04, 0xF8, 0xFF, 0x00, 0x6D, 0x6D,
//	0x6D, 0x6D, 0x6D, 0x6D, 0x6D, 0x6D, 0x00, 0xFF, 0x07, 0x08, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
//	0x0B, 0x0B, 0x08, 0x07,
//};
//
//const uint8_t battery_1_bitmap[] = {
//	0xF8, 0x04, 0x84, 0x84, 0x87, 0x87, 0x87, 0x87, 0x84, 0x84, 0x04, 0xF8, 0xFF, 0x00, 0x6D, 0x6D,
//	0x6D, 0x6D, 0x6D, 0x6D, 0x6D, 0x6D, 0x00, 0xFF, 0x07, 0x08, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
//	0x0B, 0x0B, 0x08, 0x07,
//};
//
//const uint8_t battery_2_bitmap[] = {
//	0xF8, 0x04, 0x04, 0x04, 0x07, 0x07, 0x07, 0x07, 0x04, 0x04, 0x04, 0xF8, 0xFF, 0x00, 0x6C, 0x6C,
//	0x6C, 0x6C, 0x6C, 0x6C, 0x6C, 0x6C, 0x00, 0xFF, 0x07, 0x08, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
//	0x0B, 0x0B, 0x08, 0x07,
//};
//
//const uint8_t battery_3_bitmap[] = {
//	0xF8, 0x04, 0x04, 0x04, 0x07, 0x07, 0x07, 0x07, 0x04, 0x04, 0x04, 0xF8, 0xFF, 0x00, 0x60, 0x60,
//	0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x00, 0xFF, 0x07, 0x08, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
//	0x0B, 0x0B, 0x08, 0x07,
//};
//
//const uint8_t battery_4_bitmap[] = {
//	0xF8, 0x04, 0x04, 0x04, 0x07, 0x07, 0x07, 0x07, 0x04, 0x04, 0x04, 0xF8, 0xFF, 0x00, 0x00, 0x00,
//	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x07, 0x08, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B,
//	0x0B, 0x0B, 0x08, 0x07,
//};
//
//const uint8_t battery_5_bitmap[] = {
//	0xF8, 0x04, 0x04, 0x04, 0x07, 0x07, 0x07, 0x07, 0x04, 0x04, 0x04, 0xF8, 0xFF, 0x00, 0x00, 0x00,
//	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x07, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
//	0x08, 0x08, 0x08, 0x07,
//};
//
//Image_t battery_0 = {
//	.width = 12,
//	.height = 20,
//	.bitmap = battery_0_bitmap
//};
//
//Image_t battery_1 = {
//	.width = 12,
//	.height = 20,
//	.bitmap = battery_1_bitmap
//};
//
//Image_t battery_2 = {
//	.width = 12,
//	.height = 20,
//	.bitmap = battery_2_bitmap
//};
//
//Image_t battery_3 = {
//	.width = 12,
//	.height = 20,
//	.bitmap = battery_3_bitmap
//};
//
//Image_t battery_4 = {
//	.width = 12,
//	.height = 20,
//	.bitmap = battery_4_bitmap
//};
//
//Image_t battery_5 = {
//	.width = 12,
//	.height = 20,
//	.bitmap = battery_5_bitmap
//};
//
//Image_t* battery[6] = {
//	&battery_0,
//	&battery_1,
//	&battery_2,
//	&battery_3,
//	&battery_4,
//	&battery_5
//};
