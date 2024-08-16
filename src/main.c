// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023 Brian Starkey <stark3y@gmail.com>
// modified by Artur Poznanski <apoznan5@gmail.com> 
// for Kitronik Autonomous Robotics Platform

#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "bt_hid.h"

#define PWM_MIN 0
#define PWM_MAX 100 

static inline int8_t clamp8(int16_t value) {
        if (value > 127) {
                return 127;
        } else if (value < -128) {
                return -128;
        }

        return value;
}

struct slice {
	unsigned int slice_num;
	unsigned int pwm_min;
	unsigned int pwm_max;
};

struct chassis {
	struct slice slice_l;
	struct slice slice_r;

	int8_t l;
	int8_t r;
};

void init_slice(struct slice *slice, unsigned int slice_num, unsigned int pwm_min, uint8_t pin_a)
{
	slice->slice_num = slice_num;
	slice->pwm_min = pwm_min;
	gpio_set_function(pin_a, GPIO_FUNC_PWM);
	gpio_set_function(pin_a + 1, GPIO_FUNC_PWM);
	pwm_set_wrap(slice->slice_num, slice->pwm_min + 127 + 1);
	pwm_set_chan_level(slice->slice_num, PWM_CHAN_A, 0);
	pwm_set_chan_level(slice->slice_num, PWM_CHAN_B, 0);
	pwm_set_enabled(slice->slice_num, true);
}

void chassis_init(struct chassis *chassis, uint8_t pin_la, uint8_t pin_ra)
{

	init_slice(&chassis->slice_l, pwm_gpio_to_slice_num(pin_la), PWM_MIN, pin_la);
	init_slice(&chassis->slice_r, pwm_gpio_to_slice_num(pin_ra), PWM_MIN, pin_ra);
}

static inline uint8_t abs8(int8_t v) {
	return v < 0 ? -v : v;
}

void slice_set_with_brake(struct slice *slice, int8_t value, bool brake)
{
	uint8_t mag = abs8(value);

	if (value == 0) {
		pwm_set_both_levels(slice->slice_num, brake ? slice->pwm_min + 127 : 0, brake ? slice->pwm_min + 127 : 0);
	} else if (value < 0) {
		pwm_set_both_levels(slice->slice_num, slice->pwm_min + mag, 0);
	} else {
		pwm_set_both_levels(slice->slice_num, 0, slice->pwm_min + mag);
	}
}

void slice_set(struct slice *slice, int8_t value)
{
	slice_set_with_brake(slice, value, false);
}

void chassis_set_raw(struct chassis *chassis, int8_t left, int8_t right)
{
	slice_set(&chassis->slice_l, left);
	slice_set(&chassis->slice_r, right);

	chassis->l = left;
	chassis->r = right;
}

void chassis_set(struct chassis *chassis, int8_t linear, int8_t rot)
{

	if (linear < -127) {
		linear = -127;
	}

	if (rot < -127) {
		rot = -127;
	}

	int l = linear - rot;
	int r = linear + rot;

	r = -r;
    
	chassis_set_raw(chassis, l, r);
}

void main(void) {
	stdio_init_all();

	sleep_ms(5000);
	printf("Hello\n");

	multicore_launch_core1(bt_main);
	sleep_ms(5000);

	struct chassis chassis1 = { 0 };
    	struct chassis chassis2 = { 0 };

    	chassis_init(&chassis1, 20, 6);
    	chassis_init(&chassis2, 19, 7);

	struct bt_hid_state state;
	for ( ;; ) {
		sleep_ms(20);
		bt_hid_get_latest(&state);
		
		float speed_scale1 = 0.1;
        	float speed_scale2 = 0.05;
		int8_t linear = clamp8(-(state.rx - 128) * speed_scale2);
		int8_t rot = clamp8(-(state.ly - 128) * speed_scale1);

		//printf("buttons: %04x, l: %d,%d, r: %d,%d, l2,r2: %d,%d hat: %d linear: %d, rot: %d\n",
		//		state.buttons, state.lx, state.ly, state.rx, state.ry,
		//		state.l2, state.r2, state.hat, linear, rot);
				
		chassis_set(&chassis1, linear, rot);
        	chassis_set(&chassis2, linear, rot);
	}
}