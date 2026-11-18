#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "periph.h"

#define PIN_LED 45
#define PIN_BUZZER 46
#define BUZZER_ON 1
#define BATTERY_CHANNEL ADC_CHANNEL_2      // GPIO3
#define BATTERY_SCALE 4.03f                // The board's voltage divider

#define SERVO_GROUP 1                      // Group 0 is full of motors
#define SERVO_RES_HZ 1000000
#define SERVO_PERIOD_US 20000
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
static const int SERVO_PINS[2] = {8, 21};

static adc_oneshot_unit_handle_t adc;
static adc_cali_handle_t cali;
static mcpwm_cmpr_handle_t servo_cmp[2];
static volatile int64_t buzzer_off_us;     // 0 means no timer is running
static bool led_on;

static void buzzer_task(void *arg)
{
    while (true) {
        int64_t off = buzzer_off_us;
        if (off != 0 && esp_timer_get_time() >= off) {
            gpio_set_level(PIN_BUZZER, !BUZZER_ON);
            buzzer_off_us = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void servos_init(void)
{
    mcpwm_timer_handle_t timer;
    mcpwm_timer_config_t timer_cfg = {
        .group_id = SERVO_GROUP,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_RES_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = SERVO_PERIOD_US,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timer));

    for (int i = 0; i < 2; i++) {
        mcpwm_oper_handle_t oper;
        mcpwm_operator_config_t oper_cfg = {.group_id = SERVO_GROUP};
        ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

        mcpwm_comparator_config_t cmp_cfg = {.flags.update_cmp_on_tez = true};
        ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_cfg, &servo_cmp[i]));

        mcpwm_gen_handle_t gen;
        mcpwm_generator_config_t gen_cfg = {.gen_gpio_num = SERVO_PINS[i]};
        ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_cfg, &gen));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, servo_cmp[i], MCPWM_GEN_ACTION_LOW)));
        servo_set(i, 0);
    }
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

void periph_init(void)
{
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_LED) | (1ULL << PIN_BUZZER),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&out));
    gpio_set_level(PIN_BUZZER, !BUZZER_ON);

    adc_oneshot_unit_init_cfg_t unit = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &adc));
    adc_oneshot_chan_cfg_t chan = {.bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_11};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, BATTERY_CHANNEL, &chan));
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = BATTERY_CHANNEL,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali));

    servos_init();
    xTaskCreatePinnedToCore(buzzer_task, "buzzer", 2048, NULL, 2, NULL, 0);
}

float battery_volts(void)
{
    int raw = 0, millivolts = 0;
    if (adc_oneshot_read(adc, BATTERY_CHANNEL, &raw) != ESP_OK) return 0.0f;
    if (adc_cali_raw_to_voltage(cali, raw, &millivolts) != ESP_OK) return 0.0f;
    return millivolts / 1000.0f * BATTERY_SCALE;
}

void buzzer_command(uint16_t ms)
{
    if (ms == 0) {
        buzzer_off_us = 0;
        gpio_set_level(PIN_BUZZER, !BUZZER_ON);
    } else if (ms == 1) {
        buzzer_off_us = 0;
        gpio_set_level(PIN_BUZZER, BUZZER_ON);
    } else {
        buzzer_off_us = esp_timer_get_time() + ms * 1000LL;
        gpio_set_level(PIN_BUZZER, BUZZER_ON);
    }
}

void servo_set(int servo, int degrees)
{
    if (servo < 0 || servo > 1) return;
    if (degrees < -90) degrees = -90;
    if (degrees > 90) degrees = 90;
    uint32_t us = SERVO_MIN_US + (uint32_t)(degrees + 90) * (SERVO_MAX_US - SERVO_MIN_US) / 180;
    mcpwm_comparator_set_compare_value(servo_cmp[servo], us);
}

void led_toggle(void)
{
    led_on = !led_on;
    gpio_set_level(PIN_LED, led_on);
}
