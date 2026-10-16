#ifndef FISH_SERVO_H
#define FISH_SERVO_H

#include "esp_err.h"
#include <stdint.h>

typedef enum {
    SERVO_JOINT_0 = 0,
    SERVO_JOINT_1,
    SERVO_JOINT_2,
    SERVO_COUNT
} servo_id_t;

esp_err_t fish_servo_init(void);
esp_err_t fish_servo_set_angle(servo_id_t servo, float angle_deg);
esp_err_t fish_servo_set_all(float j0_deg, float j1_deg, float j2_deg);
esp_err_t fish_servo_set_center_all(void);
esp_err_t fish_servo_disable_all(void);

#endif