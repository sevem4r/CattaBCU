/*
 * images.h
 *
 *  Created on: 09.01.2021
 *      Author: mariusz
 */

#ifndef INC_IMAGES_H_
#define INC_IMAGES_H_

#include <inttypes.h>
#include "../src/OLED_SH1106/oled_sh1106.h"

extern Image_t logo;
extern Image_t lights;
extern Image_t brakes;
extern Image_t temp;

extern Image_t* battery[15];

#endif /* INC_IMAGES_H_ */
