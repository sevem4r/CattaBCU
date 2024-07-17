/*
 * adc.h
 *
 *  Created on: 19.11.2020
 *      Author: mariusz
 */

#ifndef ADC_ADC1_H_
#define ADC_ADC1_H_

#include "common.h"

#define ADC1_CLK_ADCCLK		0
#define ADC1_CLK_PCLK2		1
#define ADC1_CLK_PCLK4		2

#define ADC1_SMP_1_5C		0
#define ADC1_SMP_7_5C		1
#define ADC1_SMP_13_5C		2
#define ADC1_SMP_28_5C		3
#define ADC1_SMP_41_5C		4
#define ADC1_SMP_55_5C		5
#define ADC1_SMP_71_5C		6
#define ADC1_SMP_293_5C		7

#define ADC1_CH0			(1<<0)
#define ADC1_CH1			(1<<1)
#define ADC1_CH2			(1<<2)
#define ADC1_CH3			(1<<3)
#define ADC1_CH4			(1<<4)
#define ADC1_CH5			(1<<5)
#define ADC1_CH6			(1<<6)
#define ADC1_CH7			(1<<7)
#define ADC1_CH8			(1<<8)
#define ADC1_CH9			(1<<9)
#define ADC1_CH10			(1<<10)
#define ADC1_CH11			(1<<11)
#define ADC1_CH12			(1<<12)
#define ADC1_CH13			(1<<13)
#define ADC1_CH14			(1<<14)
#define ADC1_CH15			(1<<15)
#define ADC1_CH16			(1<<16)
#define ADC1_CH17			(1<<17)
#define ADC1_CH18			(1<<18)

#define ADC1_CALIB_OFF		0
#define ADC1_CALIB_ON		1

#define ADC1_GPIOA			GPIOA
#define ADC1_GPIOB			GPIOB
#define ADC1_GPIOC			GPIOC

// ADC inputs map
#define ADC1_IN_NULL		-1
#define ADC1_IN0			PA0
#define ADC1_IN1			PA1
#define ADC1_IN2			PA2
#define ADC1_IN3			PA3
#define ADC1_IN4			PA4
#define ADC1_IN5			PA5
#define ADC1_IN6			PA6
#define ADC1_IN7			PA7
#define ADC1_IN8			PB0
#define ADC1_IN9			PB1
#define ADC1_IN10			PC0
#define ADC1_IN11			PC1
#define ADC1_IN12			PC2
#define ADC1_IN13			PC3
#define ADC1_IN14			PC4
#define ADC1_IN15			PC5

// USER CONFIGURATION
#define ADC1_TIMEOUT_US_INIT		1000
#define ADC1_TIMEOUT_US_CONVERSION	1000

#define ADC1_CLK_MODE		ADC1_CLK_PCLK2
#define ADC1_SAMPLING		ADC1_SMP_293_5C
#define ADC1_CALIBRATION	ADC1_CALIB_ON

#define ADC1_GPIOA_IN		ADC1_IN1
#define ADC1_GPIOB_IN		ADC1_IN1
#define ADC1_GPIOC_IN		ADC1_IN_NULL
#define ADC1_CHANNELS		ADC1_CH1 | ADC1_CH9

void adc1_set(void);
void adc1_sequecne(uint16_t* buffer, uint8_t size);

#endif /* ADC_ADC1_H_ */
