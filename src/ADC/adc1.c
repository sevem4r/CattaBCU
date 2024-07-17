/*
 * adc.c
 *
 *  Created on: 19.11.2020
 *      Author: mariusz
 */

#include "stm32f0xx.h"
#include "ADC/adc1.h"

__attribute__((weak)) uint32_t sys_tick_get(void){
	return 0UL;
}

void adc1_set(void){
	uint32_t tickstart;

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

#if ADC1_CLK_MODE == ADC1_CLK_ADCCLK
	RCC->CR2 |= RCC_CR2_HSI14ON;

	timeout_set(ADC1_TIMEOUT_US_INIT);
	while((RCC->CR2 & RCC_CR2_HSI14RDY) == 0){
		if( timeout_get() ){
			error_call(ERROR_ADC1, ERROR_CODE_ADC1_HSI_INIT);
			return;
		}
	}
#else
	// clock mode
    ADC1->CFGR2 |= ADC1_CLK_MODE;
#endif

    int16_t in_lut[3] = { ADC1_GPIOA_IN, ADC1_GPIOB_IN, ADC1_GPIOC_IN };
    GPIO_TypeDef *gpio_lut[3] = { ADC1_GPIOA, ADC1_GPIOB, ADC1_GPIOC };
    int16_t in;
    GPIO_TypeDef *gpio;
	for(uint8_t i = 0; i < 3; i++){
		in = in_lut[i];
		gpio = gpio_lut[i];
		if(in != ADC1_IN_NULL){
			for(uint8_t pin = 0; pin < 8; pin++){
				if(in & 0x01){
					gpio->OTYPER  &= ~( GPIO_CLR1B  << pin );
					gpio->PUPDR   &= ~( GPIO_CLR2B  << ( pin * 2 ) );
					gpio->OSPEEDR &= ~( GPIO_CLR2B  << ( pin * 2 ) );
					gpio->MODER   &= ~( GPIO_CLR2B  << ( pin * 2 ) );
					gpio->MODER   |=  ( GPIO_ANALOG << ( pin * 2 ) );
				}
				in >>= 1;
			}
		}
	}

	// sampling time
    ADC1->SMPR |= ADC1_SAMPLING;

    // sequence of channels to be converted
    ADC1->CHSELR |= ADC1_CHANNELS;

#if ADC1_CALIBRATION == ADC1_CALIB_ON
		// Calibration
		if((ADC1->CR & ADC_CR_ADEN) != 0){
			ADC1->CR |= ADC_CR_ADDIS;
		}

		tickstart = sys_tick_get();
		while((ADC1->CR & ADC_CR_ADEN) != 0){
			if((sys_tick_get() - tickstart) > ADC1_TIMEOUT_US_INIT){
//				error_call(ERROR_ADC1, ERROR_CODE_ADC1_ENABLE);
				return;
			}
		}

		ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
		ADC1->CR |= ADC_CR_ADCAL;

		tickstart = sys_tick_get();
		while((ADC1->CR & ADC_CR_ADCAL) != 0){
			if((sys_tick_get() - tickstart) > ADC1_TIMEOUT_US_INIT){
//				error_call(ERROR_ADC1, ERROR_CODE_ADC1_CALIBRATE);
				return;
			}
		}
#endif

    // Enable
    if((ADC1->ISR & ADC_ISR_ADRDY) != 0){
    	ADC1->ISR |= ADC_ISR_ADRDY;
    }
    ADC1->CR |= ADC_CR_ADEN;

    tickstart = sys_tick_get();
    while((ADC1->ISR & ADC_ISR_ADRDY) == 0){
		if((sys_tick_get() - tickstart) > ADC1_TIMEOUT_US_INIT){
//			error_call(ERROR_ADC1, ERROR_CODE_ADC1_ENABLE);
			return;
		}
    }
}

// Single sequence of conversions, converting all the channels once (CONT = 0 in the ADC_CFGR1)
// Conversion is started by either:
// - Setting the ADSTART bit in the ADC_CR register
// - Hardware trigger event
// Inside the sequence, after each conversion is complete:
// - The converted data are stored in the 16-bit ADC_DR register
// - The EOC (end of conversion) flag is set
// - An interrupt is generated if the EOCIE bit is set
// After the sequence of conversions is complete:
// - The EOSEQ (end of sequence) flag is set
// - An interrupt is generated if the EOSEQIE bit is set
// Then the ADC stops until a new external trigger event occurs or the ADSTART bit is set
// again.
void adc1_sequecne(uint16_t* buffer, uint8_t size){
	uint32_t tickstart;
	uint8_t index;

	if(!buffer) return;

	// Start the ADC conversion
	ADC1->CR |= ADC_CR_ADSTART;

	index = 0;

	while(1){
		// Wait end of conversion
		tickstart = sys_tick_get();
		while((ADC1->ISR & ADC_ISR_EOC) == 0){
			if((sys_tick_get() - tickstart) > ADC1_TIMEOUT_US_CONVERSION){
//				error_call(ERROR_ADC1, ERROR_CODE_ADC1_CONVERSION);
				return;
			}
		}

		// End of conversion
		if(ADC1->ISR & ADC_ISR_EOC){
			if(index < size){
				// Get converted data
				// EOC flag is cleared by reading the ADC_DR register
				buffer[index] = ADC1->DR;
			}

			index++;
		}

		// End of sequence
		if(ADC1->ISR & ADC_ISR_EOS){
			// EOS flag is cleared by software by writing 1 to it
			ADC1->ISR |= (ADC_ISR_EOS);
			break;
		}
	}
}
