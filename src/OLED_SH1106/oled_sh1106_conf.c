/*
 * oled_sh1106_conf.c
 *
 *  Created on: 13 sie 2020
 *      Author: mariusz
 */

#include "stm32f0xx.h"
#include "oled_sh1106_conf.h"
#include "main.h"
#include "common.h"
#include "SPI/spi1.h"

void _oled_sh1106_init(){

}

uint8_t _oled_sh1106_spi_transfer(uint8_t byte){
	spi1_transfer(byte);

	return 0;
}

void _oled_sh1106_delay_ms(uint16_t ms){
	sys_tick_delay_ms(ms);
}

void _oled_sh1106_cs_set(void){
	GPIO_SET(GPIOA, GPIO_OLED_CS);
}

void _oled_sh1106_cs_clr(void){
	GPIO_CLR(GPIOA, GPIO_OLED_CS);
}

void _oled_sh1106_dc_set(void){
	GPIO_SET(GPIOA, GPIO_OLED_DC);
}

void _oled_sh1106_dc_clr(void){
	GPIO_CLR(GPIOA, GPIO_OLED_DC);
}

void _oled_sh1106_res_set(void){
	GPIO_SET(GPIOA, GPIO_OLED_RES);
}

void _oled_sh1106_res_clr(void){
	GPIO_CLR(GPIOA, GPIO_OLED_RES);
}
