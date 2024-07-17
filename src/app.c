/*
 * app.c
 *
 *  Created on: 14.11.2022
 *      Author: mariusz
 */

#include "app.h"
#include "main.h"
#include "hmi.h"
#include "KEY/key.h"
#include "utility.h"

#include "stm32f0xx.h"
#include "common.h"
#include "ADC/adc1.h"

#define KEY_EVENT_LOOP_DELAY	2

// NUCLEO
// 1 - VDD_TARGET
// 2 - SWCLK
// 3 - GND
// 4 - SWDIO
// 5 - NRST
// 6 - SWO

// BOARD
// 1 - GND
// 2 - SWDIO
// 3 - SWCLK

static HmiInterface_t hmi;

enum APP_STATE{
	APP_STATE_START,
	APP_STATE_RUN,
	APP_STATE_SPECS,
	APP_STATE_CONFIG_FRONT,
	APP_STATE_CONFIG_REAR,
	APP_STATE_STOP
};

enum TRIP_STATE{
	TRIP_STATE_STOP,
	TRIP_STATE_TIME_UPDATE,
	TRIP_STATE_RUN
};

typedef struct{
	uint8_t state;
	uint32_t auto_off_counter;
	uint32_t auto_off_voltage_counter;
}App_t;

typedef struct{
	union{
		uint16_t data[32];

		struct{
			uint8_t saved;

			uint16_t diameter;
			uint16_t voltage_max;
			uint16_t voltage_min;

			uint8_t front_boost;
			uint8_t front_normal;

			uint8_t rear_boost;
			uint8_t rear_normal;

			uint32_t distance_mm;

			uint16_t total_distance_km;
			uint8_t top_speed;

			RTC_time_t timestamp;
		}rover;
	};
}Mem_t;

typedef struct{
	volatile uint8_t debounce;
	uint8_t state;
	uint32_t diameter_mm;
	uint32_t distance_mm;
	uint32_t distance;
	uint32_t speed;
	uint16_t speed_time;

	uint16_t total_distance_km;
	uint8_t top_speed;

	RTC_time_t rtc_timestamp;
	RTC_time_t rtc_time;
}Trip_t;

typedef struct{
	uint16_t voltage;
	uint16_t voltage_min;
	uint16_t voltage_max;
	uint16_t value;

	volatile IIR_filter_t filter;
}Battery_t;

typedef struct{
	uint8_t enabled;

	uint8_t front_pwm;
	int16_t front_boost;
	int16_t front_normal;
	uint8_t front_btn;

	uint8_t rear_pwm;
	int16_t rear_boost;
	int16_t rear_normal;
	uint8_t rear_btn;

	uint8_t select;
}Lights_t;

typedef struct{
	int16_t value;

	volatile IIR_filter_t filter;
}Temperature_t;

typedef struct{
	int32_t Rt;
	int16_t temp;
}NTC_t;

const NTC_t ntc_ref[43] = {
	{4879070, -550},
	{3464229, -500},
	{2477981, -450},
	{1785436, -400},
	{1295555, -350},
	{946674, -300},
	{695647, -250},
	{515872, -200},
	{384196, -150},
	{288612, -100},
	{217755, -50},
	{165642, 0},
	{126876, 50},
	{97905, 100},
	{76088, 150},
	{59530, 200},
	{47000, 250},
	{36824, 300},
	{29314, 350},
	{23470, 400},
	{18902, 450},
	{15301, 500},
	{12408, 550},
	{10117, 600},
	{8315, 650},
	{6869, 700},
	{5685, 750},
	{4724, 800},
	{3936, 850},
	{3291, 900},
	{2765, 950},
	{2339, 1000},
	{1980, 1050},
	{1682, 1100},
	{1433, 1150},
	{1225, 1200},
	{1049, 1250},
	{901, 1300},
	{776, 1350},
	{669, 1400},
	{581, 1450},
	{505, 1500},
	{441, 1550}
};

static App_t app;
static Mem_t memory;
static Temperature_t temperature;
static Battery_t battery;
static Lights_t lights;
static Trip_t trip;

void adc_conversion(void);
void lights_adjustment(void);

void auto_off_lock(void);
void auto_off_unlock(void);

void write_to_mem_default(void);
void write_to_mem(void);
void read_from_mem(void);

KeyState_t key_left_get_state(void);
KeyState_t key_center_get_state(void);
KeyState_t key_right_get_state(void);
KeyState_t key_lights_get_state(void);
KeyState_t key_brakes_get_state(void);

void key_left_action(Key_t* key, KeyAction_t action);
void key_center_action(Key_t* key, KeyAction_t action);
void key_right_action(Key_t* key, KeyAction_t action);
void key_lights_action(Key_t* key, KeyAction_t action);
void key_brakes_action(Key_t* key, KeyAction_t action);

void app_init(void){
	auto_off_lock();

	read_from_mem();

	if(memory.rover.saved != 1){
		write_to_mem_default();
	}

	trip.rtc_time.hours = memory.rover.timestamp.hours;
	trip.rtc_time.minutes = memory.rover.timestamp.minutes;
	trip.rtc_time.seconds = memory.rover.timestamp.seconds;

	trip.rtc_timestamp.hours = trip.rtc_time.hours;
	trip.rtc_timestamp.minutes = trip.rtc_time.minutes;
	trip.rtc_timestamp.seconds = trip.rtc_time.seconds;

	trip.distance_mm = memory.rover.distance_mm;
	trip.distance = trip.distance_mm / 1000;
	trip.diameter_mm = memory.rover.diameter;
	battery.voltage_min = memory.rover.voltage_min;
	battery.voltage_max = memory.rover.voltage_max;
	lights.front_boost = memory.rover.front_boost;
	lights.front_normal = memory.rover.front_normal;
	lights.rear_boost = memory.rover.rear_boost;
	lights.rear_normal = memory.rover.rear_normal;
	IIR_filter_init(&battery.filter, 95, 5);
	IIR_filter_init(&temperature.filter, 95, 5);

	hmi_init();

	key_set(KEY_ID_LEFT, key_left_get_state, 10, 1);
	key_set(KEY_ID_CENTER, key_center_get_state, 10, 10);
	key_set(KEY_ID_RIGHT, key_right_get_state, 10, 1);
	key_set(KEY_ID_LIGHTS, key_lights_get_state, 10, 10);
	key_set(KEY_ID_BRAKES, key_brakes_get_state, 10, 10);

	key_set_action(KEY_ID_LEFT, key_left_action);
	key_set_action(KEY_ID_CENTER, key_center_action);
	key_set_action(KEY_ID_RIGHT, key_right_action);
	key_set_action(KEY_ID_LIGHTS, key_lights_action);
	key_set_action(KEY_ID_BRAKES, key_brakes_action);

	app.state = APP_STATE_START;
}

void app_loop(void){
	uint8_t loop_delay = KEY_EVENT_LOOP_DELAY;

	while(1){
		loop_delay--;
		if(!loop_delay){
			key_event();
			loop_delay = KEY_EVENT_LOOP_DELAY;
		}

		switch(app.state){
		case APP_STATE_START:
			hmi_start_screen();

			sys_tick_delay_ms(1000);

			app.state = APP_STATE_RUN;
			break;

		case APP_STATE_RUN:
			adc_conversion();

			lights_adjustment();

			if(trip.state == TRIP_STATE_TIME_UPDATE){
				trip.state = TRIP_STATE_RUN;

				rtc_set_time(&trip.rtc_timestamp);
			}
			if(trip.state == TRIP_STATE_RUN){
				rtc_get_time(&trip.rtc_time);
			}

			hmi.distance = trip.distance;
			hmi.speed = trip.speed;

			hmi.hours = trip.rtc_time.hours;
			hmi.minutes = trip.rtc_time.minutes;
			hmi.seconds = trip.rtc_time.seconds;

			hmi.temperature = temperature.value;
			hmi.battery = battery.value;

			hmi.control_lights = lights.enabled;
			hmi.control_brakes = lights.rear_btn;

			hmi_run_screen(&hmi);

			break;

		case APP_STATE_SPECS:
			hmi.total_distance_km = trip.total_distance_km;
			hmi.top_speed_kmh = trip.top_speed;

			hmi_specs_screen(&hmi);
			break;

		case APP_STATE_CONFIG_FRONT:
			lights_adjustment();

			hmi.pwm_front_normal = lights.front_normal;
			hmi.pwm_front_boost = lights.front_boost;
			hmi.pwm_select = lights.select;

			hmi_config_front_screen(&hmi);
			break;

		case APP_STATE_CONFIG_REAR:
			lights_adjustment();

			hmi.pwm_rear_normal = lights.rear_normal;
			hmi.pwm_rear_boost = lights.rear_boost;
			hmi.pwm_select = lights.select;

			hmi_config_rear_screen(&hmi);
			break;

		case APP_STATE_STOP:
			hmi_start_screen();

			write_to_mem();

			sys_tick_delay_ms(1000);

			auto_off_unlock();
			break;
		}

		sys_tick_delay_ms(50);
	}
}

uint32_t adc_voltage(uint16_t value){
	uint32_t voltage;

	voltage = 2 * value * 3300 / 4096;

	return voltage;
}

int16_t adc_temperature(uint16_t value){
	int32_t R;
	int32_t Rt;
	NTC_t ntc1, ntc2;

	R = 10000;
	Rt = (value * R) / (4096 - value);

	ntc1 = ntc_ref[0];
	ntc2 = ntc_ref[0];

	for(uint8_t i = 0; i < 43; i++){
		if(Rt > ntc_ref[i].Rt){

			if(i){
				ntc1 = ntc_ref[i - 1];
				ntc2 = ntc_ref[i];
			}
			else{
				ntc1 = ntc_ref[i];
				ntc2 = ntc_ref[i];
			}

			break;
		}
	}

	int16_t temp = ntc1.temp + ((ntc2.temp - ntc1.temp) * (Rt - ntc1.Rt)) / (ntc2.Rt - ntc1.Rt);

	return temp;
}

void adc_conversion(void){
	static uint16_t ADC_raw[2];

	adc1_sequecne(ADC_raw, 2);

	IIR_filter_update(&temperature.filter, ADC_raw[0]);
	IIR_filter_update(&battery.filter, ADC_raw[1]);

	battery.voltage = adc_voltage(battery.filter.value);

	battery.voltage = clamp(battery.voltage, battery.voltage_min, battery.voltage_max);

	battery.value = (battery.voltage - battery.voltage_min) * 100 / (battery.voltage_max - battery.voltage_min);

	temperature.value = adc_temperature(temperature.filter.value);

	if(battery.voltage <= battery.voltage_min){
		app.auto_off_voltage_counter++;

		if(app.auto_off_voltage_counter >= 100){
			app.state = APP_STATE_STOP;
		}
	}
}

void lights_adjustment(void){
	uint8_t battery_correction;
	uint16_t voltage_delta;
	uint16_t voltage_delta_total;

	voltage_delta = battery.voltage - battery.voltage_min;
	voltage_delta_total = battery.voltage_max - battery.voltage_min;

	battery_correction = 40 - (voltage_delta * 40 / voltage_delta_total);

	if(lights.enabled){
		lights.front_pwm = (lights.front_btn ? lights.front_boost : lights.front_normal) + battery_correction;
		lights.rear_pwm = (lights.rear_btn ? lights.rear_boost : lights.rear_normal) + battery_correction;
	}
	else{
		lights.front_pwm = 0;
		lights.rear_pwm = 0;
	}

	pwm_front_set(lights.front_pwm);
	pwm_rear_set(lights.rear_pwm);
}

// speedometer interrupt
void exti_PA9_callback(void){
	// 100Hz max
	if(!trip.debounce){
		uint8_t speed_km;

		if(trip.state == TRIP_STATE_STOP){
			trip.state = TRIP_STATE_TIME_UPDATE;
		}

		app.auto_off_counter = 0;

		trip.distance_mm += trip.diameter_mm;

		trip.distance = trip.distance_mm / 1000;

		trip.speed_time = TIM17->CNT;
		TIM17->CNT = 0;
		trip.speed = (trip.diameter_mm * 3600) / trip.speed_time; // m/h

		// update top_speed
		speed_km = trip.speed / 1000;

		if(speed_km > trip.top_speed){
			trip.top_speed = speed_km;
		}

		// 10sec timeout reset
		TIM14->CNT = 0;

		// 10ms timer debounce
		trip.debounce = 1;

		TIM1->CNT = 0;
		TIM1->CR1 |= TIM_CR1_CEN;
	}
}

// speedometer 10ms debounce
void tim1_ovf_callback(void){
	trip.debounce = 0;

	TIM1->CR1 &= ~TIM_CR1_CEN;
}

// trip 10s timeout
void tim14_ovf_callback(void){
	trip.state = TRIP_STATE_STOP;

	trip.rtc_timestamp = trip.rtc_time;

	app.auto_off_counter++;

	// app 5min auto off (6(10s) * 5)
	if(app.auto_off_counter >= 30){
		app.state = APP_STATE_STOP;
	}
}

// speedometer time overflow
void tim17_ovf_callback(void){
	trip.speed = 0;
}

void auto_off_lock(void){
	GPIOA->ODR |= (1<<GPIO_AUTO_OFF);
}

void auto_off_unlock(void){
	GPIOA->ODR &= ~(1<<GPIO_AUTO_OFF);
}

KeyState_t key_left_get_state(void){
	return !(GPIOB->IDR & (1<<GPIO_KEY_LEFT)) ? KEY_STATE_HIGH : KEY_STATE_LOW;
}

KeyState_t key_center_get_state(void){
	return !(GPIOB->IDR & (1<<GPIO_KEY_CENTER)) ? KEY_STATE_HIGH : KEY_STATE_LOW;
}

KeyState_t key_right_get_state(void){
	return !(GPIOA->IDR & (1<<GPIO_KEY_RIGHT)) ? KEY_STATE_HIGH : KEY_STATE_LOW;
}

KeyState_t key_lights_get_state(void){
	return !(GPIOA->IDR & (1<<GPIO_LIGHTS)) ? KEY_STATE_HIGH : KEY_STATE_LOW;
}

KeyState_t key_brakes_get_state(void){
	return !(GPIOA->IDR & (1<<GPIO_BRAKES)) ? KEY_STATE_HIGH : KEY_STATE_LOW;
}

void key_left_action(Key_t* key, KeyAction_t action){
	if(action & KEY_ACTION_PRESS){
		if(app.state == APP_STATE_CONFIG_FRONT && lights.select){
			if(lights.select == 1){
				lights.front_normal -= 1; if(lights.front_normal < 0) lights.front_normal = 0;
			}
			else if(lights.select == 2){
				lights.front_boost -= 1; if(lights.front_boost < 0) lights.front_boost = 0;
			}
		}
		else if(app.state == APP_STATE_CONFIG_REAR && lights.select){
			if(lights.select == 1){
				lights.rear_normal -= 1; if(lights.rear_normal < 0) lights.rear_normal = 0;
			}
			else if(lights.select == 2){
				lights.rear_boost -= 1; if(lights.rear_boost < 0) lights.rear_boost = 0;
			}
		}
	}
	if(action & KEY_ACTION_LONG_REPEAT){
		if(app.state == APP_STATE_CONFIG_FRONT && lights.select){
			if(lights.select == 1){
				lights.front_normal -= 5; if(lights.front_normal < 0) lights.front_normal = 0;
			}
			else if(lights.select == 2){
				lights.front_boost -= 5; if(lights.front_boost < 0) lights.front_boost = 0;
			}
		}
		else if(app.state == APP_STATE_CONFIG_REAR && lights.select){
			if(lights.select == 1){
				lights.rear_normal -= 5; if(lights.rear_normal < 0) lights.rear_normal = 0;
			}
			else if(lights.select == 2){
				lights.rear_boost -= 5; if(lights.rear_boost < 0) lights.rear_boost = 0;
			}
		}
	}
	else if(action & KEY_ACTION_LONG_PRESS){
		if(app.state == APP_STATE_RUN){
			trip.state = TRIP_STATE_STOP;

			trip.total_distance_km += trip.distance / 1000;

			trip.distance_mm = 0;
			trip.distance = 0;

			trip.rtc_time.hours = 0;
			trip.rtc_time.minutes = 0;
			trip.rtc_time.seconds = 0;

			trip.rtc_timestamp.hours = 0;
			trip.rtc_timestamp.minutes = 0;
			trip.rtc_timestamp.seconds = 0;
		}
	}
}

void key_center_action(Key_t* key, KeyAction_t action){
	if(action & KEY_ACTION_PRESS){
		if(app.state == APP_STATE_CONFIG_FRONT || app.state == APP_STATE_CONFIG_REAR){
			lights.select = (lights.select < 2) ? lights.select + 1 : 0;
		}
	}
	else if(action & KEY_ACTION_LONG_PRESS){
		if(app.state == APP_STATE_RUN){
			app.state = APP_STATE_STOP;
		}
	}
}

void key_right_action(Key_t* key, KeyAction_t action){
	if(action & KEY_ACTION_PRESS){
		if(app.state == APP_STATE_CONFIG_FRONT && lights.select){
			if(lights.select == 1){
				lights.front_normal += 1; if(lights.front_normal > 200) lights.front_normal = 200;
			}
			else if(lights.select == 2){
				lights.front_boost += 1; if(lights.front_boost > 200) lights.front_boost = 200;
			}
		}
		else if(app.state == APP_STATE_CONFIG_REAR && lights.select){
			if(lights.select == 1){
				lights.rear_normal += 1; if(lights.rear_normal > 200) lights.rear_normal = 200;
			}
			else if(lights.select == 2){
				lights.rear_boost += 1; if(lights.rear_boost > 200) lights.rear_boost = 200;
			}
		}
		else{
			if(app.state == APP_STATE_RUN){
				lights.select = 0;
				app.state = APP_STATE_SPECS;
			}
			else if(app.state == APP_STATE_SPECS){
				lights.select = 0;
				app.state = APP_STATE_CONFIG_FRONT;
			}
			else if(app.state == APP_STATE_CONFIG_FRONT){
				app.state = APP_STATE_CONFIG_REAR;
			}
			else if(app.state == APP_STATE_CONFIG_REAR){
				app.state = APP_STATE_RUN;
			}
		}
	}
	if(action & KEY_ACTION_LONG_REPEAT){
		if(app.state == APP_STATE_CONFIG_FRONT && lights.select){
			if(lights.select == 1){
				lights.front_normal += 5; if(lights.front_normal > 200) lights.front_normal = 200;
			}
			else if(lights.select == 2){
				lights.front_boost += 5; if(lights.front_boost > 200) lights.front_boost = 200;
			}
		}
		else if(app.state == APP_STATE_CONFIG_REAR && lights.select){
			if(lights.select == 1){
				lights.rear_normal += 5; if(lights.rear_normal > 200) lights.rear_normal = 200;
			}
			else if(lights.select == 2){
				lights.rear_boost += 5; if(lights.rear_boost > 200) lights.rear_boost = 200;
			}
		}
	}
}

void key_lights_action(Key_t* key, KeyAction_t action){
	if(action & KEY_ACTION_RELEASE){
		if(lights.enabled){
			if(lights.front_btn){
				lights.front_btn = 0;
			}
			else{
				lights.front_btn = 1;
			}
		}
		else{
			lights.enabled = 1;
			lights.front_btn = 0;
		}
	}
	else if(action & KEY_ACTION_LONG_PRESS){
		lights.enabled = 0;
		lights.front_btn = 0;
	}
}

void key_brakes_action(Key_t* key, KeyAction_t action){
	if(action & KEY_ACTION_PRESSED){
		lights.rear_btn = 0;
	}
	else if(action & KEY_ACTION_RELEASED){
		lights.rear_btn = 1;
	}
}

// total flash size: 32768bytes, 32 pages (1024 bytes each)
// reserve last page for user data
// LinkerScript.ld
// ROM (rx)		: ORIGIN = 0x8000000, LENGTH = 31K
void write_to_mem_default(void){
	memory.rover.saved = 1;

	memory.rover.diameter = 2050;
	memory.rover.voltage_min = 3400;
	memory.rover.voltage_max = 4100;

	memory.rover.front_boost = 200;
	memory.rover.front_normal = 100;

	memory.rover.rear_boost = 200;
	memory.rover.rear_normal = 60;

	memory.rover.distance_mm = 0;

	memory.rover.timestamp.hours = 0;
	memory.rover.timestamp.minutes = 0;
	memory.rover.timestamp.seconds = 0;

	memory.rover.total_distance_km = trip.total_distance_km;
	memory.rover.top_speed = trip.top_speed;

	flash_unlock();
	flash_page_erase(0x8007C00);
	flash_write(0x8007C00, memory.data, 32);
	flash_lock();
}

void write_to_mem(void){
	memory.rover.saved = 1;

	memory.rover.diameter = trip.diameter_mm;
	memory.rover.voltage_min = battery.voltage_min;
	memory.rover.voltage_max = battery.voltage_max;

	memory.rover.front_boost = lights.front_boost;
	memory.rover.front_normal = lights.front_normal;

	memory.rover.rear_boost = lights.rear_boost;
	memory.rover.rear_normal = lights.rear_normal;

	memory.rover.distance_mm = trip.distance_mm;

	memory.rover.timestamp.hours = trip.rtc_time.hours;
	memory.rover.timestamp.minutes = trip.rtc_time.minutes;
	memory.rover.timestamp.seconds = trip.rtc_time.seconds;

	memory.rover.total_distance_km = trip.total_distance_km;
	memory.rover.top_speed = trip.top_speed;

	flash_unlock();
	flash_page_erase(0x8007C00);
	flash_write(0x8007C00, memory.data, 32);
	flash_lock();
}

void read_from_mem(void){
	flash_read(0x8007C00, memory.data, 32);
}
