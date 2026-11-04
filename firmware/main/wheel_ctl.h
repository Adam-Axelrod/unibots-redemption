// Speed control and fault detection for one wheel. Pure C, tested on a laptop.
#pragma once
#include <stdbool.h>

typedef struct {
    float kp, ki;
    int dead_zone;        // PWM added so the motor actually starts to turn
    int pwm_limit;
    int runaway_pulses;   // Pulses against the command before faulting
    int stall_steps;      // Steps at the PWM limit with no movement before faulting
} wheel_cfg_t;

typedef enum { WHEEL_OK = 0, WHEEL_RUNAWAY, WHEEL_STALL } wheel_fault_t;

typedef struct {
    float integral;
    int wrong_way;
    int stalled;
    wheel_fault_t fault;
} wheel_state_t;

// One control step. Target and measured are encoder pulses per step. Returns the
// signed PWM to apply, which is 0 for a zero target or once a fault has latched.
int wheel_step(const wheel_cfg_t *cfg, wheel_state_t *st, float target, int measured);
