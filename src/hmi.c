/*
 * hmi.c
 *
 *  Created on: 15.11.2022
 *      Author: mariusz
 */

#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hmi.h"
#include "images.h"
#include "font.h"
#include "OLED_SH1106/oled_sh1106.h"
#include "OLED_SH1106/oled_sh1106_widget.h"

typedef struct{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;

	char formatted[16];

	Widget_t widget;
}Time_t;

typedef struct{
	uint32_t value;
	uint16_t km;
	uint16_t m;

	char formatted[16];

	Widget_t widget;
}Distance_t;

typedef struct{
	uint16_t value;
	uint8_t kmh;
	uint8_t mh;

	char formatted_kmh[16];
	char formatted_mh[16];

	Widget_t widget_kmh;
	Widget_t widget_mh;
	Widget_t widget;
}Speed_t;

typedef struct{
	int16_t value;

	char formatted[16];

	Widget_t widget;
}Temperature_t;

typedef struct{
	uint8_t value;

	char formatted[16];

	Widget_t widget;
}Battery_t;

typedef struct{
	uint8_t brakes;
	uint8_t lights;
}Controls_t;

typedef struct{
	uint16_t km;

	char formatted[32];

	Widget_t widget;
}TotalDistance_t;

typedef struct{
	uint8_t kmh;

	char formatted[24];

	Widget_t widget;
}TopSpeed_t;

typedef struct{
	uint8_t value;

	char formatted[20];

	Widget_t widget;
}Pwm_t;

typedef struct{
	Time_t time;
	Distance_t distance;
	Speed_t speed;
	Temperature_t temperature;
	Battery_t battery;
	Controls_t controls;

	TotalDistance_t total_distance;
	TopSpeed_t top_speed;
	Widget_t title;

	Pwm_t pwm_normal;
	Pwm_t pwm_boost;
}Hmi_t;

static Hmi_t hmi;

void hmi_init(void){
	oled_sh1106_init();
}

void hmi_start_screen(void){
	oled_sh1106_clear();
	oled_sh1106_draw_bitmap(&logo, 10, 10);
	oled_sh1106_display();
}

void hmi_stop_screen(void){
	oled_sh1106_clear();
	oled_sh1106_draw_bitmap(&logo, 10, 10);
	oled_sh1106_display();
}

void hmi_run_screen(HmiInterface_t* interface){
	hmi.controls.brakes = interface->control_brakes;
	hmi.controls.lights = interface->control_lights;

	hmi.battery.value = interface->battery / (100 / (15 - 1));

	sprintf(hmi.time.formatted, "%02d:%02d:%02d", interface->hours, interface->minutes, interface->seconds);

	oled_sh1106_text_widget(
		&hmi.time.widget,
		2,
		2,
		hmi.time.formatted,
		&font8,
		LCD_WMODE_LEFT,
		LCD_HMODE_UP,
		NULL
	);

	hmi.distance.value = interface->distance;
	hmi.distance.km = hmi.distance.value / 1000;
	hmi.distance.m = hmi.distance.value - hmi.distance.km * 1000;

	sprintf(hmi.distance.formatted, "%d.%03d", hmi.distance.km, hmi.distance.m);

	oled_sh1106_text_widget(
		&hmi.distance.widget,
		125,
		2,
		hmi.distance.formatted,
		&font8,
		LCD_WMODE_RIGHT,
		LCD_HMODE_UP,
		NULL
	);

	hmi.speed.value = interface->speed;
	hmi.speed.kmh = hmi.speed.value / 1000;
	hmi.speed.mh = (hmi.speed.value - hmi.speed.kmh * 1000) / 100;

	itoa(hmi.speed.kmh, hmi.speed.formatted_kmh, 10);
	itoa(hmi.speed.mh, hmi.speed.formatted_mh, 10);

	oled_sh1106_text_widget(
		&hmi.speed.widget_kmh,
		63,
		16,
		hmi.speed.formatted_kmh,
		&font34,
		LCD_WMODE_CENTER,
		LCD_HMODE_UP,
		NULL
	);

	oled_sh1106_text_widget(
		&hmi.speed.widget_mh,
		hmi.speed.widget_kmh.size.width + 1,
		0,
		hmi.speed.formatted_mh,
		&font16,
		LCD_WMODE_LEFT,
		LCD_HMODE_UP,
		&hmi.speed.widget_kmh
	);

	oled_sh1106_text_widget(
		&hmi.speed.widget,
		hmi.speed.widget_kmh.size.width / 2,
		hmi.speed.widget_kmh.size.height - 11,
		"km/h",
		&font8,
		LCD_WMODE_CENTER,
		LCD_HMODE_UP,
		&hmi.speed.widget_kmh
	);

	if(interface->temperature > (hmi.temperature.value + 5)){
		hmi.temperature.value = interface->temperature;
	}
	if(interface->temperature < (hmi.temperature.value - 5)){
		hmi.temperature.value = interface->temperature;
	}

	itoa(hmi.temperature.value / 10, hmi.temperature.formatted, 10);

	oled_sh1106_text_widget(
		&hmi.temperature.widget,
		115,
		52,
		hmi.temperature.formatted,
		&font8,
		LCD_WMODE_CENTER,
		LCD_HMODE_UP,
		NULL
	);

	oled_sh1106_clear();

	oled_sh1106_draw_widget(&hmi.time.widget);
	oled_sh1106_draw_widget(&hmi.distance.widget);
	oled_sh1106_draw_widget(&hmi.speed.widget_kmh);
	oled_sh1106_draw_widget(&hmi.temperature.widget);
	oled_sh1106_draw_bitmap(battery[hmi.battery.value], 2, 24);
	if(hmi.controls.brakes){
		oled_sh1106_draw_bitmap(&brakes, 16, 50);
	}
	if(hmi.controls.lights){
		oled_sh1106_draw_bitmap(&lights, 2, 50);
	}
	oled_sh1106_draw_bitmap(&temp, 108, 24);

	oled_sh1106_display();
}

void hmi_specs_screen(HmiInterface_t* interface){

	// title
	oled_sh1106_text_widget(
		&hmi.title,
		LCD_WIDTH / 2,
		5,
		"Specs",
		&font8,
		LCD_WMODE_CENTER,
		LCD_HMODE_UP,
		NULL
	);

	// total_distance
	hmi.total_distance.km = interface->total_distance_km;

	sprintf(hmi.total_distance.formatted, "Distance: %d km", hmi.total_distance.km);

	oled_sh1106_text_widget(
		&hmi.total_distance.widget,
		LCD_WIDTH / 2,
		20,
		hmi.total_distance.formatted,
		&font8,
		LCD_WMODE_CENTER,
		LCD_HMODE_UP,
		NULL
	);

	// top_speed
	hmi.top_speed.kmh = interface->top_speed_kmh;

	sprintf(hmi.top_speed.formatted, "Speed: %d km/h", hmi.top_speed.kmh);

	oled_sh1106_text_widget(
		&hmi.top_speed.widget,
		LCD_WIDTH / 2,
		35,
		hmi.top_speed.formatted,
		&font8,
		LCD_WMODE_CENTER,
		LCD_HMODE_UP,
		NULL
	);

	oled_sh1106_clear();

	oled_sh1106_draw_widget(&hmi.title);
	oled_sh1106_draw_widget(&hmi.total_distance.widget);
	oled_sh1106_draw_widget(&hmi.top_speed.widget);

	oled_sh1106_display();
}

void hmi_config_front_screen(HmiInterface_t* interface){

	uint8_t select = interface->pwm_select;

	// title
	oled_sh1106_text_widget(
		&hmi.title,
		LCD_WIDTH / 2,
		5,
		"Config Front",
		&font8,
		LCD_WMODE_CENTER,
		LCD_HMODE_UP,
		NULL
	);

	// front_pwm_normal
	hmi.pwm_normal.value = interface->pwm_front_normal;

	sprintf(hmi.pwm_normal.formatted, (select == 1) ? ">Front normal: %d" : "Front normal: %d", hmi.pwm_normal.value);

	oled_sh1106_text_widget(
		&hmi.pwm_normal.widget,
		LCD_WIDTH / 2,
		20,
		hmi.pwm_normal.formatted,
		&font8,
		LCD_WMODE_CENTER,
		LCD_HMODE_UP,
		NULL
	);

	// front_pwm_boost
	hmi.pwm_boost.value = interface->pwm_front_boost;

	sprintf(hmi.pwm_boost.formatted, (select == 2) ? ">Front boost: %d" : "Front boost: %d", hmi.pwm_boost.value);

	oled_sh1106_text_widget(
		&hmi.pwm_boost.widget,
		LCD_WIDTH / 2,
		35,
		hmi.pwm_boost.formatted,
		&font8,
		LCD_WMODE_CENTER,
		LCD_HMODE_UP,
		NULL
	);

	oled_sh1106_clear();

	oled_sh1106_draw_widget(&hmi.title);
	oled_sh1106_draw_widget(&hmi.pwm_normal.widget);
	oled_sh1106_draw_widget(&hmi.pwm_boost.widget);

	oled_sh1106_display();
}

void hmi_config_rear_screen(HmiInterface_t* interface){

	uint8_t select = interface->pwm_select;

	// title
	oled_sh1106_text_widget(
		&hmi.title,
		LCD_WIDTH / 2,
		5,
		"Config Rear",
		&font8,
		LCD_WMODE_CENTER,
		LCD_HMODE_UP,
		NULL
	);

	// rear_pwm_normal
	hmi.pwm_normal.value = interface->pwm_rear_normal;

	sprintf(hmi.pwm_normal.formatted, (select == 1) ? ">Rear normal: %d" : "Rear normal: %d", hmi.pwm_normal.value);

	oled_sh1106_text_widget(
		&hmi.pwm_normal.widget,
		LCD_WIDTH / 2,
		20,
		hmi.pwm_normal.formatted,
		&font8,
		LCD_WMODE_CENTER,
		LCD_HMODE_UP,
		NULL
	);

	// rear_pwm_boost
	hmi.pwm_boost.value = interface->pwm_rear_boost;

	sprintf(hmi.pwm_boost.formatted, (select == 2) ? ">Rear boost: %d" : "Rear boost: %d", hmi.pwm_boost.value);

	oled_sh1106_text_widget(
		&hmi.pwm_boost.widget,
		LCD_WIDTH / 2,
		35,
		hmi.pwm_boost.formatted,
		&font8,
		LCD_WMODE_CENTER,
		LCD_HMODE_UP,
		NULL
	);

	oled_sh1106_clear();

	oled_sh1106_draw_widget(&hmi.title);
	oled_sh1106_draw_widget(&hmi.pwm_normal.widget);
	oled_sh1106_draw_widget(&hmi.pwm_boost.widget);

	oled_sh1106_display();
}
