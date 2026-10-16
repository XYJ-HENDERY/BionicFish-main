#include "bmi270_driver.h"
#include "fish_config.h"
#include "software_i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG_BMI = "bmi270";

static uint8_t s_bmi270_i2c_addr = BMI270_I2C_ADDR;

/* BMI270 register addresses */
#define BMI270_REG_CHIP_ID      0x00
#define BMI270_REG_CMD          0x7E
#define BMI270_REG_PWR_CONF     0x7C
#define BMI270_REG_PWR_CTRL     0x7D
#define BMI270_REG_ACC_CONF     0x40
#define BMI270_REG_ACC_RANGE    0x41
#define BMI270_REG_GYR_CONF     0x42
#define BMI270_REG_GYR_RANGE    0x43
#define BMI270_REG_ACC_DATA     0x0C
#define BMI270_REG_GYR_DATA     0x12

#define BMI270_SOFT_RESET_CMD   0xB6

static esp_err_t bmi270_reg_read(uint8_t reg, uint8_t *data, uint8_t len)
{
    esp_err_t ret = software_i2c_write(s_bmi270_i2c_addr, &reg, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    return software_i2c_read(s_bmi270_i2c_addr, data, len);
}

static esp_err_t bmi270_reg_write(uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t buf[33];
    if (len > sizeof(buf) - 1) {
        return ESP_ERR_INVALID_ARG;
    }
    buf[0] = reg;
    for (uint8_t i = 0; i < len; i++) {
        buf[i + 1] = data[i];
    }
    return software_i2c_write(s_bmi270_i2c_addr, buf, len + 1);
}

static esp_err_t bmi270_reg_write_byte(uint8_t reg, uint8_t val)
{
    return bmi270_reg_write(reg, &val, 1);
}

static esp_err_t i2c_soft_master_init(void)
{
    software_i2c_config_t config = {
        .sda_gpio = I2C_MASTER_SDA_IO,
        .scl_gpio = I2C_MASTER_SCL_IO,
        .delay_us = 5,
    };

    return software_i2c_init(&config);
}

static void gpio_interrupt_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BMI270_INT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&io_conf);
}

esp_err_t bmi270_sensor_init(void)
{
    ESP_LOGI(TAG_BMI, "=== BMI270 sensor init start (direct register mode) ===");
    ESP_LOGI(TAG_BMI, "Pins: SDA=GPIO%d, SCL=GPIO%d, INT=GPIO%d",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, BMI270_INT_PIN);

    esp_err_t ret = i2c_soft_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BMI, "Soft I2C master init failed: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG_BMI, "Soft I2C master init OK");

    int sda_level = gpio_get_level(I2C_MASTER_SDA_IO);
    int scl_level = gpio_get_level(I2C_MASTER_SCL_IO);
    ESP_LOGI(TAG_BMI, "I2C bus idle state: SDA=%d SCL=%d (both should be HIGH=1)",
             sda_level, scl_level);
    if (sda_level == 0) {
        ESP_LOGE(TAG_BMI, "SDA is stuck LOW! Check for short to GND or missing pull-up.");
    }
    if (scl_level == 0) {
        ESP_LOGE(TAG_BMI, "SCL is stuck LOW! Check for short to GND or missing pull-up.");
    }

    ESP_LOGI(TAG_BMI, "GPIO toggle test: toggling SDA and SCL...");
    gpio_set_level(I2C_MASTER_SDA_IO, 0);
    gpio_set_level(I2C_MASTER_SCL_IO, 0);
    esp_rom_delay_us(100);
    int sda_low = gpio_get_level(I2C_MASTER_SDA_IO);
    int scl_low = gpio_get_level(I2C_MASTER_SCL_IO);
    gpio_set_level(I2C_MASTER_SDA_IO, 1);
    gpio_set_level(I2C_MASTER_SCL_IO, 1);
    esp_rom_delay_us(100);
    int sda_high = gpio_get_level(I2C_MASTER_SDA_IO);
    int scl_high = gpio_get_level(I2C_MASTER_SCL_IO);
    ESP_LOGI(TAG_BMI, "GPIO toggle result: SDA(LOW=%d HIGH=%d) SCL(LOW=%d HIGH=%d)",
             sda_low, sda_high, scl_low, scl_high);
    if (sda_low != 0 || sda_high != 1 || scl_low != 0 || scl_high != 1) {
        ESP_LOGE(TAG_BMI, "GPIO control FAILED! Pins may be reserved or not connected.");
    } else {
        ESP_LOGI(TAG_BMI, "GPIO control OK");
    }

    bool device_found = false;
    uint32_t speed_table[] = {5, 10, 25, 50, 100};

    for (int speed_idx = 0; speed_idx < sizeof(speed_table) / sizeof(speed_table[0]); speed_idx++) {
        uint32_t delay_us = speed_table[speed_idx];
        software_i2c_set_delay_us(delay_us);
        ESP_LOGI(TAG_BMI, "Trying I2C at ~%lu kHz (delay=%lu us)...",
                 1000000 / (delay_us * 2), delay_us);

        software_i2c_scan();

        bool probe_0x68 = software_i2c_probe(BMI270_I2C_ADDR);
        bool probe_0x69 = software_i2c_probe(0x69);
        ESP_LOGI(TAG_BMI, "Probe 0x68=%s  0x69=%s",
                 probe_0x68 ? "ACK" : "NACK",
                 probe_0x69 ? "ACK" : "NACK");

        if (probe_0x68 || probe_0x69) {
            device_found = true;
            if (!probe_0x68 && probe_0x69) {
                s_bmi270_i2c_addr = 0x69;
            }
            ESP_LOGI(TAG_BMI, "I2C slave device found at address: 0x%02X", s_bmi270_i2c_addr);
            break;
        }
    }

    if (!device_found) {
        ESP_LOGE(TAG_BMI, "BMI270 not detected at any speed!");
        return ESP_FAIL;
    }

    uint8_t chip_id = 0;
    ret = bmi270_reg_read(BMI270_REG_CHIP_ID, &chip_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BMI, "Failed to read chip ID");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG_BMI, "Chip ID: 0x%02X (expected 0x%02X for BMI270)", chip_id, 0x24);

    if (chip_id != 0x24) {
        ESP_LOGE(TAG_BMI, "Chip ID mismatch! Got 0x%02X, expected 0x24. Aborting.", chip_id);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG_BMI, "Sending soft reset...");
    ret = bmi270_reg_write_byte(BMI270_REG_CMD, BMI270_SOFT_RESET_CMD);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BMI, "Soft reset failed: %d", ret);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG_BMI, "Soft reset OK");
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG_BMI, "Configuring ACCEL: 100Hz, OSR4, ±4g");
    ret = bmi270_reg_write_byte(BMI270_REG_ACC_CONF, 0x26);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BMI, "ACC_CONF write failed: %d", ret);
        return ESP_FAIL;
    }
    ret = bmi270_reg_write_byte(BMI270_REG_ACC_RANGE, 0x01);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BMI, "ACC_RANGE write failed: %d", ret);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG_BMI, "Configuring GYRO: 100Hz, OSR4, ±2000dps");
    ret = bmi270_reg_write_byte(BMI270_REG_GYR_CONF, 0x26);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BMI, "GYR_CONF write failed: %d", ret);
        return ESP_FAIL;
    }
    ret = bmi270_reg_write_byte(BMI270_REG_GYR_RANGE, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BMI, "GYR_RANGE write failed: %d", ret);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG_BMI, "Enabling sensors (ACCEL + GYRO + TEMP)...");
    ret = bmi270_reg_write_byte(BMI270_REG_PWR_CONF, 0x02);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BMI, "PWR_CONF write failed: %d", ret);
        return ESP_FAIL;
    }
    ret = bmi270_reg_write_byte(BMI270_REG_PWR_CTRL, 0x07);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BMI, "PWR_CTRL write failed: %d", ret);
        return ESP_FAIL;
    }

    gpio_interrupt_init();

    ESP_LOGI(TAG_BMI, "=== BMI270 sensor init SUCCESS ===");
    return ESP_OK;
}

esp_err_t bmi270_read_sensor(bmi270_data_t *data)
{
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t raw[12];
    esp_err_t ret = bmi270_reg_read(BMI270_REG_ACC_DATA, raw, 12);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    int16_t acc_x = (int16_t)(raw[1] << 8 | raw[0]);
    int16_t acc_y = (int16_t)(raw[3] << 8 | raw[2]);
    int16_t acc_z = (int16_t)(raw[5] << 8 | raw[4]);
    int16_t gyr_x = (int16_t)(raw[7] << 8 | raw[6]);
    int16_t gyr_y = (int16_t)(raw[9] << 8 | raw[8]);
    int16_t gyr_z = (int16_t)(raw[11] << 8 | raw[10]);

    data->acc_x = acc_x / 8192.0f;
    data->acc_y = acc_y / 8192.0f;
    data->acc_z = acc_z / 8192.0f;
    data->gyro_x = gyr_x / 16.384f;
    data->gyro_y = gyr_y / 16.384f;
    data->gyro_z = gyr_z / 16.384f;
    data->temp = 0.0f;

    return ESP_OK;
}

esp_err_t bmi270_calculate_attitude(bmi270_data_t *sensor_data, attitude_t *attitude)
{
    if (!sensor_data || !attitude) {
        return ESP_ERR_INVALID_ARG;
    }

    float ax = sensor_data->acc_x;
    float ay = sensor_data->acc_y;
    float az = sensor_data->acc_z;

    float norm = sqrt(ax*ax + ay*ay + az*az);
    if (norm > 0.001f) {
        ax /= norm;
        ay /= norm;
        az /= norm;
    }

    attitude->pitch = atan2(-ax, az) * 180.0f / (float)M_PI;
    attitude->roll = atan2(ay, az) * 180.0f / (float)M_PI;
    attitude->yaw = 0.0f;

    return ESP_OK;
}

bool bmi270_check_interrupt(void)
{
    return gpio_get_level(BMI270_INT_PIN) == 1;
}