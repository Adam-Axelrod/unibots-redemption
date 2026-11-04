#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "config.h"
#include "wheel_ctl.h"
#include "motors.h"

#define K_M ((TRACK_WIDTH_M + WHEEL_BASE_M) / 2.0f)
#define METRES_PER_PULSE (3.14159265f * WHEEL_DIAMETER_M / (float)ENCODER_PULSES_PER_REV)
#define PULSE_LIMIT 30000

// Left front, left rear, right front, right rear: motor ports 1 to 4.
static const struct { int pwm_a, pwm_b, enc_a, enc_b, group; } PINS[4] = {
    {4, 5, 6, 7, 0},
    {15, 16, 47, 48, 0},
    {9, 10, 11, 12, 0},
    {13, 14, 1, 2, 1},   // Each MCPWM group drives three motors at most
};
static const bool PWM_INVERT[4] = MOTOR_PWM_INVERT;
static const bool ENC_INVERT[4] = MOTOR_ENC_INVERT;

static bdc_motor_handle_t motor[4];
static pcnt_unit_handle_t encoder[4];

static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static float target_mps[4];
static float measured_mps[4];
static int64_t last_cmd_us;
static volatile bool faulted;

static void set_pwm(int i, int pwm)
{
    if (PWM_INVERT[i]) pwm = -pwm;
    if (pwm == 0) {
        bdc_motor_coast(motor[i]);
    } else if (pwm > 0) {
        bdc_motor_forward(motor[i]);
        bdc_motor_set_speed(motor[i], (uint32_t)pwm);
    } else {
        bdc_motor_reverse(motor[i]);
        bdc_motor_set_speed(motor[i], (uint32_t)-pwm);
    }
}

static void control_task(void *arg)
{
    const wheel_cfg_t cfg = {
        .kp = PID_KP,
        .ki = PID_KI,
        .dead_zone = PWM_DEAD_ZONE,
        .pwm_limit = PWM_LIMIT,
        .runaway_pulses = RUNAWAY_PULSES,
        .stall_steps = STALL_MS / CONTROL_PERIOD_MS,
    };
    static wheel_state_t state[4];
    int last_count[4] = {0};
    const float dt = CONTROL_PERIOD_MS / 1000.0f;
    TickType_t wake = xTaskGetTickCount();

    while (true) {
        float targets[4];
        taskENTER_CRITICAL(&lock);
        bool fresh = esp_timer_get_time() - last_cmd_us < CMD_TIMEOUT_MS * 1000LL;
        for (int i = 0; i < 4; i++) targets[i] = fresh ? target_mps[i] : 0.0f;
        taskEXIT_CRITICAL(&lock);

        float speeds[4];
        bool any_fault = false;
        for (int i = 0; i < 4; i++) {
            int count = 0;
            pcnt_unit_get_count(encoder[i], &count);
            int pulses = count - last_count[i];
            last_count[i] = count;
            if (ENC_INVERT[i]) pulses = -pulses;
            speeds[i] = pulses * METRES_PER_PULSE / dt;

            float target_pulses = targets[i] * dt / METRES_PER_PULSE;
            set_pwm(i, wheel_step(&cfg, &state[i], target_pulses, pulses));
            if (state[i].fault != WHEEL_OK) any_fault = true;
        }

        // One bad wheel makes the others drive the robot somewhere unintended.
        if (any_fault) {
            for (int i = 0; i < 4; i++) {
                state[i].fault = state[i].fault != WHEEL_OK ? state[i].fault : WHEEL_RUNAWAY;
                set_pwm(i, 0);
            }
            faulted = true;
        }

        taskENTER_CRITICAL(&lock);
        for (int i = 0; i < 4; i++) measured_mps[i] = speeds[i];
        taskEXIT_CRITICAL(&lock);

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

static void encoder_init(int i)
{
    pcnt_unit_config_t unit = {
        .high_limit = PULSE_LIMIT,
        .low_limit = -PULSE_LIMIT,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit, &encoder[i]));
    pcnt_glitch_filter_config_t filter = {.max_glitch_ns = 1000};
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(encoder[i], &filter));

    pcnt_chan_config_t a_cfg = {.edge_gpio_num = PINS[i].enc_a, .level_gpio_num = PINS[i].enc_b};
    pcnt_chan_config_t b_cfg = {.edge_gpio_num = PINS[i].enc_b, .level_gpio_num = PINS[i].enc_a};
    pcnt_channel_handle_t a, b;
    ESP_ERROR_CHECK(pcnt_new_channel(encoder[i], &a_cfg, &a));
    ESP_ERROR_CHECK(pcnt_new_channel(encoder[i], &b_cfg, &b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoder[i], PULSE_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoder[i], -PULSE_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(encoder[i]));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(encoder[i]));
    ESP_ERROR_CHECK(pcnt_unit_start(encoder[i]));
}

void motors_init(void)
{
    for (int i = 0; i < 4; i++) {
        bdc_motor_config_t cfg = {
            .pwm_freq_hz = PWM_FREQ_HZ,
            .pwma_gpio_num = PINS[i].pwm_b,   // Yahboom's mapping: B first
            .pwmb_gpio_num = PINS[i].pwm_a,
        };
        bdc_motor_mcpwm_config_t mcpwm = {
            .group_id = PINS[i].group,
            .resolution_hz = PWM_RESOLUTION_HZ,
        };
        ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&cfg, &mcpwm, &motor[i]));
        ESP_ERROR_CHECK(bdc_motor_enable(motor[i]));
        encoder_init(i);
    }
    // Pinned away from micro-ROS so serial traffic cannot delay the control loop.
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 10, NULL, 1);
}

void motors_command(const twist_t *cmd)
{
    float wheels[4];
    mecanum_inverse(cmd, K_M, MAX_WHEEL_SPEED_MPS, wheels);
    taskENTER_CRITICAL(&lock);
    for (int i = 0; i < 4; i++) target_mps[i] = wheels[i];
    last_cmd_us = esp_timer_get_time();
    taskEXIT_CRITICAL(&lock);
}

void motors_measured(twist_t *out)
{
    float wheels[4];
    taskENTER_CRITICAL(&lock);
    for (int i = 0; i < 4; i++) wheels[i] = measured_mps[i];
    taskEXIT_CRITICAL(&lock);
    mecanum_forward(wheels, K_M, out);
}

bool motors_faulted(void)
{
    return faulted;
}
