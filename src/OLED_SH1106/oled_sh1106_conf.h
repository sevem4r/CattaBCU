/*
 * oled_sh1106_conf.h
 *
 *  Created on: 13 sie 2020
 *      Author: mariusz
 */

#ifndef OLED_SH1106_OLED_SH1106_CONF_H_
#define OLED_SH1106_OLED_SH1106_CONF_H_

void _oled_sh1106_init();
uint8_t _oled_sh1106_spi_transfer(uint8_t byte);
void _oled_sh1106_delay_ms(uint16_t ms);
void _oled_sh1106_cs_set(void);
void _oled_sh1106_cs_clr(void);
void _oled_sh1106_dc_set(void);
void _oled_sh1106_dc_clr(void);
void _oled_sh1106_res_set(void);
void _oled_sh1106_res_clr(void);

#endif /* OLED_SH1106_OLED_SH1106_CONF_H_ */
