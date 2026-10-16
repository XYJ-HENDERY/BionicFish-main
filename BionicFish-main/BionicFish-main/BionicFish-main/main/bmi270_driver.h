#ifndef BMI270_DRIVER_H
#define BMI270_DRIVER_H

#include "esp_err.h"
#include <stdbool.h>

typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp;
} bmi270_data_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} attitude_t;

esp_err_t bmi270_sensor_init(void);
esp_err_t bmi270_read_sensor(bmi270_data_t *data);
esp_err_t bmi270_calculate_attitude(bmi270_data_t *sensor_data, attitude_t *attitude);
bool bmi270_check_interrupt(void);

#endif