/*
 * utility.h
 *
 *  Created on: 29.11.2022
 *      Author: mariusz
 */

#ifndef UTILITY_H_
#define UTILITY_H_

#include <stdint.h>

typedef struct{
	uint8_t warmup;
	int32_t a;
	int32_t b;
	int32_t divider;

	int32_t value;
}IIR_filter_t;

int32_t clamp(int32_t value, int32_t min, int32_t max);
void IIR_filter_init(volatile IIR_filter_t* filter, int32_t a, int32_t b);
void IIR_filter_update(volatile IIR_filter_t* filter, int32_t value);

#endif /* UTILITY_H_ */
