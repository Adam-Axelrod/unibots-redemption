// Tuning and limits. Physical measurements are in chassis.h. Change, rebuild, reflash.
#pragma once
#include <stdbool.h>
#include "chassis.h"

#define WHEEL_DIAMETER_M   (WHEEL_DIAMETER_MM / 1000.0f)
#define TRACK_WIDTH_M      (TRACK_WIDTH_MM / 1000.0f)
#define WHEEL_BASE_M       (WHEEL_BASE_MM / 1000.0f)

#define MAX_WHEEL_SPEED_MPS  1.0f     // Wheel commands are scaled down to fit this

// ---- Speed control. Starting values, to be tuned with the wheels lifted. --
#define CONTROL_PERIOD_MS    20
#define PWM_FREQ_HZ          25000
#define PWM_RESOLUTION_HZ    10000000
#define PWM_MAX              (PWM_RESOLUTION_HZ / PWM_FREQ_HZ)   // 400 ticks = 100%
#define PWM_LIMIT            240      // Deliberately capped at 60% for first runs
#define PWM_DEAD_ZONE        100      // Motors need 3-5 V before they start to turn
#define PID_KP               6.0f
#define PID_KI               1.0f

// ---- Safety ---------------------------------------------------------------
#define CMD_TIMEOUT_MS       500      // Stop if /cmd_vel goes quiet
#define RUNAWAY_PULSES       400      // Pulses the wrong way before we give up
#define STALL_MS             2000     // At the PWM limit with no movement
#define LOW_BATTERY_V        6.4f     // Yahboom's cut-off is 5.6 V

// ---- micro-ROS ------------------------------------------------------------
#define ROS_DOMAIN_ID        20
#define ROS_BAUD             921600
