/*
 * hmi.h
 *
 *  Created on: 15.11.2022
 *      Author: mariusz
 */

#ifndef HMI_H_
#define HMI_H_

#include <stdint.h>

typedef struct{
	uint8_t control_lights;
	uint8_t control_brakes;

	uint8_t battery;
	int16_t temperature;

	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;

	uint32_t distance;
	uint32_t speed;

	uint32_t total_distance_km;
	uint16_t top_speed_kmh;

	uint8_t pwm_front_normal;
	uint8_t pwm_front_boost;
	uint8_t pwm_rear_normal;
	uint8_t pwm_rear_boost;
	uint8_t pwm_select;
}HmiInterface_t;

void hmi_init(void);
void hmi_start_screen(void);
void hmi_stop_screen(void);
void hmi_run_screen(HmiInterface_t* interface);
void hmi_specs_screen(HmiInterface_t* interface);
void hmi_config_front_screen(HmiInterface_t* interface);
void hmi_config_rear_screen(HmiInterface_t* interface);

#endif /* HMI_H_ */
