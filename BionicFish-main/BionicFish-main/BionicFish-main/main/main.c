#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"

#include "fish_config.h"
#include "fish_servo.h"
#include "fish_motion.h"
#include "fish_control.h"
#include "ph_sensor.h"
#include "wifi_connector.h"
#include "cloud_client.h"
#include "local_server.h"
#include "esp_log.h"

static const char *TAG = "main";

static fish_control_t g_fish_ctrl;
static QueueHandle_t g_servo_queue;
static bool g_bmi270_ok = false;

typedef struct {
    float j0_deg;
    float j1_deg;
    float j2_deg;
} servo_cmd_t;

static void servo_task(void *arg)
{
    servo_cmd_t cmd;

    while (1) {
        if (xQueueReceive(g_servo_queue, &cmd, pdMS_TO_TICKS(50)) == pdTRUE) {
            fish_servo_set_all(cmd.j0_deg, cmd.j1_deg, cmd.j2_deg);
        }
    }
}

static void control_task(void *arg)
{
    fish_control_init(&g_fish_ctrl);

    if (!g_bmi270_ok) {
        fish_control_set_demo_mode(&g_fish_ctrl, true);
        ESP_LOGW(TAG, "BMI270 not available, entering demo mode");
    }

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS);

    while (1) {
        fish_control_update(&g_fish_ctrl);

        servo_cmd_t cmd = {
            .j0_deg = g_fish_ctrl.joint_angles[0],
            .j1_deg = g_fish_ctrl.joint_angles[1],
            .j2_deg = g_fish_ctrl.joint_angles[2],
        };
        xQueueOverwrite(g_servo_queue, &cmd);

        vTaskDelayUntil(&last_wake, period);
    }
}

static void cloud_upload_task(void *arg)
{
    while (!wifi_connector_is_connected()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "Cloud upload task started");

    while (1) {
        if (wifi_connector_is_connected()) {
            fish_snapshot_t snap;
            fish_control_get_snapshot(&g_fish_ctrl, &snap);

            esp_err_t ret = cloud_upload_sensor(snap.ph_data.ph_value,
                                                 snap.attitude.roll,
                                                 snap.attitude.pitch);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Upload failed");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CLOUD_UPLOAD_INTERVAL_MS));
    }
}

static void cloud_poll_task(void *arg)
{
    while (!wifi_connector_is_connected()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "Cloud poll task started");

    while (1) {
        if (wifi_connector_is_connected()) {
            cloud_cmd_t cmd;
            esp_err_t ret = cloud_poll_command(&cmd);
            if (ret == ESP_OK && cmd != CLOUD_CMD_NONE) {
                fish_control_set_external_cmd(&g_fish_ctrl, (external_cmd_t)cmd);
            }
            /* 空指令/网络错误: 不清除摇杆指令, 避免覆盖本地方向键 */
        }

        vTaskDelay(pdMS_TO_TICKS(CLOUD_POLL_INTERVAL_MS));
    }
}

static void ph_log_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(PH_READ_PERIOD_MS));

        ph_data_t ph_data;
        esp_err_t ret = ph_sensor_read(&ph_data);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "pH read failed: %d", ret);
            continue;
        }

        fish_control_set_ph_data(&g_fish_ctrl, &ph_data);

        fish_snapshot_t snap;
        fish_control_get_snapshot(&g_fish_ctrl, &snap);

        ESP_LOGI(TAG, "pH=%.2f  V=%.3fV  T=%.1fC(Tv=%.3fV)  DO=%d  state=%s",
                 ph_data.ph_value,
                 ph_data.voltage,
                 ph_data.temperature,
                 ph_data.raw_temp_volt,
                 ph_data.digital_level,
                 fish_state_name(snap.state));
    }
}

void app_main(void)
{
    /* ---------- NVS 初始化（WiFi 必需） ---------- */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* ---------- 传感器初始化 ---------- */
    ret = bmi270_sensor_init();
    if (ret != ESP_OK) {
        g_bmi270_ok = false;
        ESP_LOGW(TAG, "BMI270 init failed");
    } else {
        g_bmi270_ok = true;
    }

    ret = ph_sensor_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "pH sensor init failed");
    } else {
        ESP_LOGI(TAG, "pH sensor initialized");
    }

    ret = fish_servo_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Servo init failed, aborting");
        return;
    }

    /* ---------- WiFi + 云端初始化 ---------- */
    ret = wifi_connector_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi connect failed, cloud features unavailable");
    } else {
        cloud_client_init();
    }

    /* ---------- 本地 HTTP 控制服务（方向键用） ---------- */
    local_server_set_control(&g_fish_ctrl);
    ret = local_server_start();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Local HTTP server start failed");
    }

    /* ---------- FreeRTOS 任务 ---------- */
    g_servo_queue = xQueueCreate(1, sizeof(servo_cmd_t));
    if (!g_servo_queue) {
        ESP_LOGE(TAG, "Queue create failed");
        return;
    }

    xTaskCreate(control_task, "fish_control",
                TASK_STACK_CONTROL, NULL, TASK_PRIO_CONTROL, NULL);

    xTaskCreate(servo_task, "fish_servo",
                TASK_STACK_SERVO, NULL, TASK_PRIO_SERVO, NULL);

    xTaskCreate(ph_log_task, "ph_log",
                TASK_STACK_CONTROL, NULL, TASK_PRIO_PH_SENSOR, NULL);

    xTaskCreate(cloud_upload_task, "cloud_up",
                TASK_STACK_CLOUD, NULL, TASK_PRIO_CLOUD_UPLOAD, NULL);

    xTaskCreate(cloud_poll_task, "cloud_poll",
                TASK_STACK_CLOUD, NULL, TASK_PRIO_CLOUD_POLL, NULL);
}