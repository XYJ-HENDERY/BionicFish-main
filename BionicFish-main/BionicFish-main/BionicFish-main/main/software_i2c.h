#ifndef SOFTWARE_I2C_H
#define SOFTWARE_I2C_H

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int sda_gpio;
    int scl_gpio;
    uint32_t delay_us;
} software_i2c_config_t;

esp_err_t software_i2c_init(const software_i2c_config_t *config);
void software_i2c_set_delay_us(uint32_t delay_us);
esp_err_t software_i2c_write(uint8_t addr, const uint8_t *data, size_t len);
esp_err_t software_i2c_read(uint8_t addr, uint8_t *data, size_t len);
bool software_i2c_probe(uint8_t addr);
void software_i2c_scan(void);

#endif