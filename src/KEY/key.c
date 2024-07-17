/*
 * key.c
 *
 *  Created on: 2 maj 2017
 *      Author: Mariusz
 */

#include <stddef.h>
#include <stdbool.h>
#include "key.h"

enum KEY_RELEASE_TYPE{
	KEY_RELEASE_TYPE_NORMAL,
	KEY_RELEASE_TYPE_LONG
};

Key_t* key_get(
	KeyID_t ID)
{
	static Key_t keys[KEY_ID_MAX];

	return ID < KEY_ID_MAX ? &keys[ID] : 0;
}

void key_set(
	KeyID_t ID,
	key_get_state_callback get_state,
	uint16_t ticks_wait,
	uint16_t ticks_repeat)
{
	Key_t* key = key_get(ID);

	key->release_type = KEY_RELEASE_TYPE_NORMAL;

	key->ticks_wait = ticks_wait;
	key->ticks_repeat = ticks_repeat;

	key->callbakck_get_state = get_state;
	key->callback_action = NULL;
}

void key_set_action(
	KeyID_t ID,
	key_action_callback action)
{
	Key_t* key = key_get(ID);

	key->callback_action = action;
}

void key_event(
	void)
{
	for(uint8_t i = 0; i < KEY_ID_MAX; i++){
		Key_t* key = key_get(i);
		if(key->callback_action && key->callbakck_get_state){
			uint8_t action = KEY_ACTION_NONE;
			bool reset = false;

			key->state = key->callbakck_get_state();

			action |= key->state ? KEY_ACTION_PRESSED : KEY_ACTION_RELEASED;

			if(key->state == KEY_STATE_HIGH && key->state_prev == KEY_STATE_LOW){
				action |= KEY_ACTION_PRESS;
				reset = true;
			}
			else if(key->state == KEY_STATE_LOW && key->state_prev == KEY_STATE_HIGH){
				if(key->release_type == KEY_RELEASE_TYPE_NORMAL){
					action |= KEY_ACTION_RELEASE;
				}
				else{
					action |= KEY_ACTION_LONG_RELEASE;
				}

				reset = true;
			}
			else if(key->state == key->state_prev && key->state == KEY_STATE_HIGH){
				if(key->ticks == 0){
					if(key->release_type == KEY_RELEASE_TYPE_NORMAL){
						action |= KEY_ACTION_LONG_PRESS;
						key->release_type = KEY_RELEASE_TYPE_LONG;
					}
					else{
						action |= KEY_ACTION_LONG_REPEAT;
					}
					key->ticks = key->ticks_repeat;
				}
				else{
					key->ticks--;
				}
			}

			if(reset){
				key->ticks = key->ticks_wait;
				key->release_type = KEY_RELEASE_TYPE_NORMAL;
			}

			key->callback_action(key, action);

			key->state_prev = key->state;
		}
	}
}
