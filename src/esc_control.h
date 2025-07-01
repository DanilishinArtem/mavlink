// esc_control.h
#ifndef ESC_CONTROL_H
#define ESC_CONTROL_H

#include "driver/ledc.h"

#define ESC_GPIO 7
#define ESC_LEDC_CHANNEL LEDC_CHANNEL_0
#define ESC_LEDC_TIMER LEDC_TIMER_0
#define ESC_LEDC_MODE LEDC_LOW_SPEED_MODE
#define ESC_FREQ_HZ 50 // 50 Гц для ESC (стандартная частота)
#define ESC_RESOLUTION LEDC_TIMER_10_BIT

bool esc_initialized = false;

void esc_init() {
    printf("[ESC] Starting esc_init\n");
    if (esc_initialized) {
        printf("[ESC] Already initialized\n");
        return;
    }

    esp_err_t err;

    ledc_timer_config_t ledc_timer = {
        .speed_mode = ESC_LEDC_MODE,
        .duty_resolution = ESC_RESOLUTION,
        .timer_num = ESC_LEDC_TIMER,
        .freq_hz = ESC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false
    };
    err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        printf("[FAILED] ledc_timer_config failed: %s\n", esp_err_to_name(err));
        return;
    }else{
        printf("[ESC] Timer config success\n");
    }

    ledc_channel_config_t ledc_channel = {
        .gpio_num = ESC_GPIO,
        .speed_mode = ESC_LEDC_MODE,
        .channel = ESC_LEDC_CHANNEL,
        .timer_sel = ESC_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) {
        printf("[FAILED] ledc_channel_config failed: %s\n", esp_err_to_name(err));
        return;
    }else{
        printf("[ESC] Channel config success\n");
    }

    esc_initialized = true;
    printf("[ESC] esc_init success\n");
}

void esc_set_speed(float speed) {
    if (!esc_initialized) {
        printf("[ESC] esc_set_speed called before init!\n");
        return;
    }
    // Скорость от -1.0 до 1.0 (ноль — стоп)
    // 1.0 ms = 51, 1.5 ms = 77, 2.0 ms = 102 (для 10-битного PWM и 50 Гц)
    const int duty_min = 51;
    const int duty_neutral = 77;
    const int duty_max = 102;

    int duty = duty_neutral;
    if (speed > 0)
        duty = duty_neutral + (int)((duty_max - duty_neutral) * ((speed * 2) - 1));
    else if (speed < 0)
        duty = duty_neutral + (int)((duty_neutral - duty_min) * ((speed * 2) - 1));

    if (duty < duty_min) duty = duty_min;
    if (duty > duty_max) duty = duty_max;

    ledc_set_duty(ESC_LEDC_MODE, ESC_LEDC_CHANNEL, duty);
    ledc_update_duty(ESC_LEDC_MODE, ESC_LEDC_CHANNEL);
}

#endif
