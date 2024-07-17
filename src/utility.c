/*
 * utility.c
 *
 *  Created on: 29.11.2022
 *      Author: mariusz
 */

#include "utility.h"

int32_t clamp(int32_t value, int32_t min, int32_t max){
	if(value < min){
		value = min;
	}
	else if(value > max){
		value = max;
	}

	return value;
}

void IIR_filter_init(volatile IIR_filter_t* filter, int32_t a, int32_t b){
	filter->warmup = 0;
	filter->a = a;
	filter->b = b;
	filter->divider = filter->a + filter->b;

	filter->value = 0;
}

void IIR_filter_update(volatile IIR_filter_t* filter, int32_t value){
	if(filter->warmup){
		filter->value = (filter->value * filter->a + value * filter->b) / filter->divider;
	}
	else{
		filter->warmup++;
		filter->value = value;
	}
}
