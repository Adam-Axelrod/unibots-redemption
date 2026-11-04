// The small things: battery voltage, buzzer, servos, status LED.
#pragma once
#include <stdint.h>

void periph_init(void);
float battery_volts(void);
void buzzer_command(uint16_t ms);     // 0 off, 1 stays on, otherwise on for that long
void servo_set(int servo, int degrees);   // servo 0 or 1, -90 to 90
void led_toggle(void);
