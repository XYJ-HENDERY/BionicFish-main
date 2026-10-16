#ifndef PH_SENSOR_H
#define PH_SENSOR_H

#include "esp_err.h"
#include <stdbool.h>

typedef struct {
    float ph_value;
    float voltage;
    float temperature;
    float raw_temp_volt;
    bool  digital_level;
} ph_data_t;

esp_err_t ph_sensor_init(void);
esp_err_t ph_sensor_read(ph_data_t *data);
esp_err_t ph_sensor_calibrate(float slope, float offset);
esp_err_t ph_sensor_temp_calibrate(float slope, float offset);
void ph_sensor_get_calibration(float *slope, float *offset);
void ph_sensor_get_temp_calibration(float *slope, float *offset);

#endif