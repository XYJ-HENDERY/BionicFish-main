#include "fish_servo.h"
#include "fish_config.h"
#include "driver/ledc.h"
#include <math.h>

typedef struct {
    servo_id_t id;
    int gpio;
    ledc_channel_t channel;
    const char *name;
} servo_config_t;

static const servo_config_t servo_cfgs[SERVO_COUNT] = {
    [SERVO_JOINT_0] = { .id = SERVO_JOINT_0, .gpio = SERVO_JOINT_0_PIN, .channel = LEDC_CH_JOINT_0, .name = "Joint-0(Near)" },
    [SERVO_JOINT_1] = { .id = SERVO_JOINT_1, .gpio = SERVO_JOINT_1_PIN, .channel = LEDC_CH_JOINT_1, .name = "Joint-1(Mid)"  },
    [SERVO_JOINT_2] = { .id = SERVO_JOINT_2, .gpio = SERVO_JOINT_2_PIN, .channel = LEDC_CH_JOINT_2, .name = "Joint-2(Far)"  },
};

static float angle_to_duty(float angle_deg)
{
    float clamped = fmaxf(-SERVO_MAX_ANGLE_DEG, fminf(SERVO_MAX_ANGLE_DEG, angle_deg));
    float pulse_us = SERVO_CENTER_US + (clamped / SERVO_MAX_ANGLE_DEG) * (SERVO_MAX_PULSE_US - SERVO_CENTER_US);
    float period_us = 1000000.0f / LEDC_FREQ_HZ;
    float duty = (pulse_us / period_us) * ((1 << LEDC_RESOLUTION) - 1);
    return duty;
}

esp_err_t fish_servo_init(void)
{

    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_RESOLUTION,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) {
        return ret;
    }

    for (int i = 0; i < SERVO_COUNT; i++) {
        ledc_channel_config_t ch_cfg = {
            .gpio_num = servo_cfgs[i].gpio,
            .speed_mode = LEDC_MODE,
            .channel = servo_cfgs[i].channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER,
            .duty = 0,
            .hpoint = 0,
            .flags = { .output_invert = 0 },
        };
        ret = ledc_channel_config(&ch_cfg);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    fish_servo_set_center_all();
    return ESP_OK;
}

esp_err_t fish_servo_set_angle(servo_id_t servo, float angle_deg)
{
    if (servo >= SERVO_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    float duty = angle_to_duty(angle_deg);
    uint32_t duty_int = (uint32_t)duty;

    esp_err_t ret = ledc_set_duty(LEDC_MODE, servo_cfgs[servo].channel, duty_int);
    if (ret != ESP_OK) {
        return ret;
    }
    return ledc_update_duty(LEDC_MODE, servo_cfgs[servo].channel);
}

esp_err_t fish_servo_set_all(float j0_deg, float j1_deg, float j2_deg)
{
    esp_err_t ret;
    ret = fish_servo_set_angle(SERVO_JOINT_0, j0_deg + SERVO_TRIM_J0);
    if (ret != ESP_OK) return ret;
    ret = fish_servo_set_angle(SERVO_JOINT_1, j1_deg + SERVO_TRIM_J1);
    if (ret != ESP_OK) return ret;
    ret = fish_servo_set_angle(SERVO_JOINT_2, j2_deg + SERVO_TRIM_J2);
    return ret;
}

esp_err_t fish_servo_set_center_all(void)
{
    return fish_servo_set_all(0.0f, 0.0f, 0.0f);
}

esp_err_t fish_servo_disable_all(void)
{
    esp_err_t ret;
    for (int i = 0; i < SERVO_COUNT; i++) {
        ret = ledc_stop(LEDC_MODE, servo_cfgs[i].channel, 0);
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}