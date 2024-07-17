/*
 * common.c
 *
 *  Created on: 19.11.2020
 *      Author: mariusz
 */

#include "stm32f0xx.h"
#include "common.h"

#define ERROR_LIST_SIZE		16

typedef struct{
	uint8_t src;
	uint8_t code;
}Error_msg_t;

static uint8_t error_list_size;
static Error_msg_t error_list[ERROR_LIST_SIZE];

static TIM_TypeDef *tim_timeout;
static void (*error_call_callback)(uint8_t src, uint8_t code);
static void (*error_show_callback)(uint8_t src, uint8_t code);

void set_error_event(void (*callback)(uint8_t src, uint8_t code)){
	error_call_callback = callback;
}

void set_error_show_event(void (*callback)(uint8_t src, uint8_t code)){
	error_show_callback = callback;
}

void error_call(uint8_t src, uint8_t code){
	if(error_list_size < ERROR_LIST_SIZE){
		error_list[error_list_size].src = src;
		error_list[error_list_size].code = code;
		error_list_size++;
	}else{
		for(uint8_t i=1; i<ERROR_LIST_SIZE; i++){
			error_list[i-1] = error_list[i];
		}
		error_list[ERROR_LIST_SIZE - 1].src = src;
		error_list[ERROR_LIST_SIZE - 1].code = code;
	}

	if(error_call_callback){
		error_call_callback(src, code);
	}
}

void error_show(){
	if(error_show_callback){
		for(uint8_t i=0; i<error_list_size; i++){
			error_show_callback(error_list[i].src, error_list[i].code);
		}
	}
}

uint8_t errors_number(){
	return error_list_size;
}

void gpio_set_in(GPIO_TypeDef *port, uint8_t pin, uint8_t pull){
	if(port == GPIOA) RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	else if(port == GPIOB) RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	else if(port == GPIOC) RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	else if(port == GPIOF) RCC->AHBENR |= RCC_AHBENR_GPIOFEN;

	port->MODER |= (GPIO_INPUT << (pin * 2));
	port->PUPDR |= (GPIO_NO_PULL << (pin * 2));
	port->OTYPER |= (pull << (pin * 2));
}

void gpio_set_out(GPIO_TypeDef *port, uint8_t pin){
	if(port == GPIOA) RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	else if(port == GPIOB) RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	else if(port == GPIOC) RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	else if(port == GPIOF) RCC->AHBENR |= RCC_AHBENR_GPIOFEN;

	port->MODER |= (GPIO_OUTPUT << (pin * 2));
	port->PUPDR |= (GPIO_NO_PULL << (pin * 2));
	port->OTYPER |= (GPIO_PP << (pin * 2));
}

void timeout_register_tim(TIM_TypeDef *tim){
	tim_timeout = tim;
}

void timeout_set(uint32_t time){
	tim_timeout->CNT = time;
	tim_timeout->SR &= ~TIM_SR_UIF;
}

uint8_t timeout_get(){
	if( TIM17->SR & TIM_SR_UIF ){
		TIM17->SR &= ~TIM_SR_UIF;
		return 1;
	}
	return 0;
}
