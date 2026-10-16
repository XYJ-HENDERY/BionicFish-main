#include "software_i2c.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "soft_i2c";

static int s_sda_gpio = -1;
static int s_scl_gpio = -1;
static uint32_t s_delay_us = 5;

#define SDA_HIGH() gpio_set_level(s_sda_gpio, 1)
#define SDA_LOW()  gpio_set_level(s_sda_gpio, 0)
#define SCL_HIGH() gpio_set_level(s_scl_gpio, 1)
#define SCL_LOW()  gpio_set_level(s_scl_gpio, 0)
#define SDA_READ() gpio_get_level(s_sda_gpio)

static void sda_set_input(void)
{
    gpio_set_direction(s_sda_gpio, GPIO_MODE_INPUT);
}

static void sda_set_output(void)
{
    gpio_set_direction(s_sda_gpio, GPIO_MODE_INPUT_OUTPUT);
}

static void i2c_delay(void)
{
    esp_rom_delay_us(s_delay_us);
}

static void i2c_start(void)
{
    sda_set_output();
    SDA_HIGH();
    SCL_HIGH();
    i2c_delay();
    SDA_LOW();
    i2c_delay();
    SCL_LOW();
    i2c_delay();
}

static void i2c_stop(void)
{
    sda_set_output();
    SDA_LOW();
    i2c_delay();
    SCL_HIGH();
    i2c_delay();
    SDA_HIGH();
    i2c_delay();
}

static bool i2c_write_byte(uint8_t byte)
{
    sda_set_output();

    for (int i = 7; i >= 0; i--) {
        SCL_LOW();
        if (byte & (1 << i)) {
            SDA_HIGH();
        } else {
            SDA_LOW();
        }
        i2c_delay();
        SCL_HIGH();
        i2c_delay();
    }

    SCL_LOW();
    SDA_HIGH();
    i2c_delay();

    sda_set_input();
    SCL_HIGH();
    i2c_delay();

    bool ack = !SDA_READ();
    SCL_LOW();

    return ack;
}

static uint8_t i2c_read_byte(bool ack)
{
    uint8_t byte = 0;

    sda_set_input();
    SDA_HIGH();

    for (int i = 7; i >= 0; i--) {
        SCL_LOW();
        i2c_delay();
        SCL_HIGH();
        i2c_delay();
        if (SDA_READ()) {
            byte |= (1 << i);
        }
    }

    SCL_LOW();
    sda_set_output();
    if (ack) {
        SDA_LOW();
    } else {
        SDA_HIGH();
    }
    i2c_delay();
    SCL_HIGH();
    i2c_delay();
    SCL_LOW();
    SDA_HIGH();

    return byte;
}

esp_err_t software_i2c_init(const software_i2c_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }

    s_sda_gpio = config->sda_gpio;
    s_scl_gpio = config->scl_gpio;
    s_delay_us = config->delay_us;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << s_sda_gpio) | (1ULL << s_scl_gpio),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        return ret;
    }

    SDA_HIGH();
    SCL_HIGH();
    esp_rom_delay_us(10);

    for (int i = 0; i < 9; i++) {
        SCL_LOW();
        esp_rom_delay_us(10);
        SCL_HIGH();
        esp_rom_delay_us(10);
    }

    i2c_start();
    i2c_stop();

    SDA_HIGH();
    SCL_HIGH();
    esp_rom_delay_us(10);

    return ESP_OK;
}

esp_err_t software_i2c_write(uint8_t addr, const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_start();

    sda_set_output();
    if (!i2c_write_byte(addr << 1)) {
        i2c_stop();
        return ESP_FAIL;
    }

    for (size_t i = 0; i < len; i++) {
        if (!i2c_write_byte(data[i])) {
            i2c_stop();
            return ESP_FAIL;
        }
    }

    i2c_stop();
    return ESP_OK;
}

esp_err_t software_i2c_read(uint8_t addr, uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_start();

    sda_set_output();
    if (!i2c_write_byte((addr << 1) | 1)) {
        i2c_stop();
        return ESP_FAIL;
    }

    for (size_t i = 0; i < len; i++) {
        data[i] = i2c_read_byte(i < len - 1);
    }

    i2c_stop();
    return ESP_OK;
}

bool software_i2c_probe(uint8_t addr)
{
    if (addr < 1 || addr > 127) {
        return false;
    }

    i2c_start();
    bool ack = i2c_write_byte(addr << 1);
    i2c_stop();

    return ack;
}

void software_i2c_scan(void)
{
    int found = 0;
    ESP_LOGI(TAG, "I2C bus scan start (SDA=GPIO%d, SCL=GPIO%d)...", s_sda_gpio, s_scl_gpio);
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (software_i2c_probe(addr)) {
            ESP_LOGI(TAG, "  Found device at 0x%02X", addr);
            found++;
        }
        vTaskDelay(1);
    }

    if (found == 0) {
        ESP_LOGW(TAG, "No I2C devices found! Check wiring/pull-ups.");
    } else {
        ESP_LOGI(TAG, "I2C scan done: %d device(s) found", found);
    }
}

void software_i2c_set_delay_us(uint32_t delay_us)
{
    s_delay_us = delay_us;
}