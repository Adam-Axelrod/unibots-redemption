#include <stdlib.h>
#include "wheel_ctl.h"

int wheel_step(const wheel_cfg_t *cfg, wheel_state_t *st, float target, int measured)
{
    if (st->fault != WHEEL_OK) return 0;

    if (target == 0.0f) {
        st->integral = 0.0f;
        st->wrong_way = 0;
        st->stalled = 0;
        return 0;
    }

    // Moving against the command. A brief spell is normal when the command reverses
    // at speed. A lot means a backwards encoder, which positive feedback would
    // otherwise drive to full power, so stop for good.
    if ((target > 0.0f && measured < 0) || (target < 0.0f && measured > 0)) {
        st->wrong_way += abs(measured);
        if (st->wrong_way >= cfg->runaway_pulses) {
            st->fault = WHEEL_RUNAWAY;
            return 0;
        }
    } else if (measured != 0) {
        st->wrong_way = 0;
    }

    float error = target - (float)measured;
    float limit = (float)(cfg->pwm_limit - cfg->dead_zone);
    float output = cfg->kp * error + st->integral;

    // Anti-windup: only integrate while the output has room, or the error is
    // already pulling it back.
    bool saturated = (output >= limit && error > 0.0f) || (output <= -limit && error < 0.0f);
    if (!saturated) st->integral += cfg->ki * error;

    // Never drive against the commanded direction: braking is left to friction.
    if (target > 0.0f && output < 0.0f) output = 0.0f;
    if (target < 0.0f && output > 0.0f) output = 0.0f;
    if (output > limit) output = limit;
    if (output < -limit) output = -limit;

    // At the limit and not moving: unplugged encoder or jammed wheel.
    if ((output >= limit || output <= -limit) && measured == 0) {
        if (++st->stalled >= cfg->stall_steps) {
            st->fault = WHEEL_STALL;
            return 0;
        }
    } else {
        st->stalled = 0;
    }

    int pwm = (int)output;
    if (pwm > 0) return pwm + cfg->dead_zone;
    if (pwm < 0) return pwm - cfg->dead_zone;
    return 0;
}
