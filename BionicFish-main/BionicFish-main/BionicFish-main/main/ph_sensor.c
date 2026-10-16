#include "ph_sensor.h"
#include "fish_config.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

static const char *PH_TAG = "ph_sensor";

static adc_oneshot_unit_handle_t adc_handle;
static bool initialized = false;
static SemaphoreHandle_t s_mutex = NULL;

static float g_slope      = PH_SLOPE;
static float g_offset     = PH_OFFSET;
static float g_temp_slope = PH_TEMP_SLOPE;
static float g_temp_offset = PH_TEMP_OFFSET;

static float g_ph_ema   = 0.0f;
static float g_volt_ema = 0.0f;
static float g_temp_ema = 0.0f;
static bool  g_ema_init = false;

static int cmp_int(const void *a, const void *b)
{
    return (*(int *)a - *(int *)b);
}

static float adc_read_voltage_filtered(adc_channel_t channel)
{
    int samples[PH_SAMPLE_COUNT];

    for (int i = 0; i < PH_SAMPLE_COUNT; i++) {
        int raw = 0;
        esp_err_t ret = adc_oneshot_read(adc_handle, channel, &raw);
        if (ret != ESP_OK) {
            raw = 0;
        }
        samples[i] = raw;
        esp_rom_delay_us(200);
    }

    qsort(samples, PH_SAMPLE_COUNT, sizeof(int), cmp_int);

    int trim = PH_SAMPLE_COUNT / 8;
    int sum = 0;
    int count = 0;
    for (int i = trim; i < PH_SAMPLE_COUNT - trim; i++) {
        sum += samples[i];
        count++;
    }

    float avg_raw = (float)sum / count;
    float voltage = avg_raw * 3.3f / 4096.0f;
    return voltage;
}

static float ema_filter(float current, float prev, float alpha)
{
    if (!g_ema_init) return current;
    return alpha * current + (1.0f - alpha) * prev;
}

static float voltage_to_ph(float voltage, float temp_c)
{
    float ph_25c = g_slope * voltage + g_offset;
    return ph_25c + PH_TEMP_COEFF * (temp_c - 25.0f);
}

static float voltage_to_temperature(float voltage)
{
    return g_temp_slope * voltage + g_temp_offset;
}

esp_err_t ph_sensor_init(void)
{
    if (initialized) {
        return ESP_OK;
    }

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        ESP_LOGE(PH_TAG, "Mutex create failed");
        return ESP_ERR_NO_MEM;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PH_DIGITAL_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = PH_ADC_UNIT,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t ret = adc_oneshot_new_unit(&unit_cfg, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(PH_TAG, "ADC oneshot init failed: %d", ret);
        return ret;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = PH_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_oneshot_config_channel(adc_handle, PH_ADC_CHANNEL, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(PH_TAG, "pH ADC channel config failed: %d", ret);
        return ret;
    }

    chan_cfg.atten = PH_TEMP_ADC_ATTEN;
    ret = adc_oneshot_config_channel(adc_handle, PH_TEMP_ADC_CHANNEL, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(PH_TAG, "Temp ADC channel config failed: %d", ret);
        return ret;
    }

    initialized = true;

    g_ph_ema   = 7.0f;
    g_volt_ema = (7.0f - g_offset) / g_slope;
    g_temp_ema = 0.8f;
    g_ema_init = true;

    ESP_LOGI(PH_TAG, "pH sensor init OK (median+EMA filter, temp-compensated)");
    return ESP_OK;
}

esp_err_t ph_sensor_read(ph_data_t *data)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(PH_TAG, "ADC mutex timeout");
        return ESP_ERR_TIMEOUT;
    }

    memset(data, 0, sizeof(*data));

    float raw_volt = adc_read_voltage_filtered(PH_ADC_CHANNEL);
    float raw_temp_volt = adc_read_voltage_filtered(PH_TEMP_ADC_CHANNEL);

    g_volt_ema = ema_filter(raw_volt, g_volt_ema, PH_EMA_ALPHA);
    g_temp_ema = ema_filter(raw_temp_volt, g_temp_ema, PH_EMA_ALPHA);

    float temp_c = voltage_to_temperature(g_temp_ema);
    float ph_val = voltage_to_ph(g_volt_ema, temp_c);
    g_ph_ema = ema_filter(ph_val, g_ph_ema, PH_EMA_ALPHA);

    g_ema_init = true;

    data->voltage       = g_volt_ema;
    data->ph_value      = g_ph_ema;
    data->temperature   = temp_c;
    data->raw_temp_volt = g_temp_ema;
    data->digital_level = (gpio_get_level(PH_DIGITAL_PIN) == 1);

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t ph_sensor_calibrate(float slope, float offset)
{
    g_slope  = slope;
    g_offset = offset;
    g_ph_ema   = 7.0f;
    g_volt_ema = (7.0f - g_offset) / g_slope;
    g_ema_init = true;
    return ESP_OK;
}

esp_err_t ph_sensor_temp_calibrate(float slope, float offset)
{
    g_temp_slope  = slope;
    g_temp_offset = offset;
    g_temp_ema = 0.8f;
    g_ema_init = true;
    return ESP_OK;
}

void ph_sensor_get_calibration(float *slope, float *offset)
{
    if (slope)  *slope  = g_slope;
    if (offset) *offset = g_offset;
}

void ph_sensor_get_temp_calibration(float *slope, float *offset)
{
    if (slope)  *slope  = g_temp_slope;
    if (offset) *offset = g_temp_offset;
}