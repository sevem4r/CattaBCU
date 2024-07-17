#ifndef __MAIN_H
#define __MAIN_H

#include <stdint.h>

#define GPIO_OC_REAR	PA0
#define GPIO_TMEP		PA1
#define GPIO_OLED_RES	PA2
#define GPIO_OLED_DC	PA3
#define GPIO_OLED_CS	PA4
#define GPIO_OLED_SCK	PA5
#define GPIO_OLED_MISO	PA7

#define GPIO_LIGHTS		PA12
#define GPIO_BRAKES		PA11
#define GPIO_KEY_LEFT	PB4
#define GPIO_KEY_CENTER	PB3
#define GPIO_KEY_RIGHT	PA15
#define GPIO_AUTO_OFF	PA8

#define GPIO_PWM_FRONT	PA6
#define GPIO_PWM_REAR	PB5

typedef struct{
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
}RTC_time_t;

uint32_t flash_unlock(void);
uint32_t flash_lock(void);
uint32_t flash_page_erase(uint32_t address);
uint32_t flash_write(uint32_t address, uint16_t* data, uint32_t size);
uint32_t flash_read(uint32_t address, uint16_t* data, uint32_t size);

void sys_tick_delay_ms(uint32_t ms);

void rtc_set_time(RTC_time_t* time);
void rtc_get_time(RTC_time_t* time);

void pwm_front_set(uint8_t pwm);
void pwm_rear_set(uint8_t pwm);

void tim1_ovf_callback(void);
void tim14_ovf_callback(void);
void tim17_ovf_callback(void);
void exti_PA9_callback(void);

#endif /* __MAIN_H */
