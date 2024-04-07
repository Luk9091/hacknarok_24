#ifndef __ENCODER__H
#define __ENCODER__H

#define ENC_A	9
#define ENC_B	8
#define ENC_SW	7

#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/pio.h>
#include <hardware/gpio.h>
#include <stdlib.h>
#include <hardware/rtc.h>
#include <pico/util/datetime.h>

// extern uint8_t left_rotation;
// extern uint8_t right_rotation;
// extern uint8_t digit_to_change;
// extern uint8_t new_digit;

void encoder_init();
void encoder_enable_interrupts();
void encoder_callback(uint gpio, uint32_t events);
void encoder_switch_callback();
void set_time(datetime_t *time);

#endif