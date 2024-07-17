/*
 * key.h
 *
 *  Created on: 2 maj 2017
 *      Author: Mariusz
 */

#ifndef KEY_KEY_H_
#define KEY_KEY_H_

#include <stdint.h>

typedef enum KEY_ID{
	KEY_ID_LEFT,
	KEY_ID_CENTER,
	KEY_ID_RIGHT,
	KEY_ID_LIGHTS,
	KEY_ID_BRAKES,
	KEY_ID_MAX
}KeyID_t;

typedef enum KEY_ACTION{
	KEY_ACTION_NONE			= 0,
	KEY_ACTION_PRESSED		= (1<<0),
	KEY_ACTION_RELEASED		= (1<<1),
	KEY_ACTION_PRESS 		= (1<<2),
	KEY_ACTION_RELEASE		= (1<<3),
	KEY_ACTION_LONG_PRESS	= (1<<4),
	KEY_ACTION_LONG_REPEAT	= (1<<5),
	KEY_ACTION_LONG_RELEASE	= (1<<6)
}KeyAction_t;

typedef enum KEY_STATE{
	KEY_STATE_LOW,
	KEY_STATE_HIGH
}KeyState_t;

typedef struct Key Key_t;
typedef KeyState_t (*key_get_state_callback)(void);
typedef void (*key_action_callback)(Key_t* key, KeyAction_t action);

struct Key{
	KeyState_t state;
	KeyState_t state_prev;
	uint8_t release_type;
	uint16_t ticks;
	uint16_t ticks_wait;
	uint16_t ticks_repeat;

	key_get_state_callback callbakck_get_state;
	key_action_callback callback_action;
};

Key_t* key_get(KeyID_t ID);
void key_set(KeyID_t ID, key_get_state_callback get_state, uint16_t ticks_wait, uint16_t ticks_repeat);
void key_set_action(KeyID_t ID, key_action_callback action);
void key_event(void);

#endif /* KEY_KEY_H_ */
