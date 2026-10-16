#include "cloud_client.h"
#include "fish_config.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "cloud";

static char g_upload_url[128];
static char g_poll_url[128];

static const char *cmd_names[] = {
    [CLOUD_CMD_NONE]     = "NONE",
    [CLOUD_CMD_FORWARD]  = "FORWARD",
    [CLOUD_CMD_LEFT]     = "LEFT",
    [CLOUD_CMD_RIGHT]    = "RIGHT",
    [CLOUD_CMD_STOP]     = "STOP",
    [CLOUD_CMD_AUTO]     = "AUTO",
    [CLOUD_CMD_DIVE]     = "DIVE",
    [CLOUD_CMD_SURFACE]  = "SURFACE",
};

const char *cloud_cmd_name(cloud_cmd_t cmd)
{
    if (cmd > CLOUD_CMD_SURFACE) return "UNKNOWN";
    return cmd_names[cmd];
}

static cloud_cmd_t parse_command_string(const char *str)
{
    if (!str || strlen(str) == 0) return CLOUD_CMD_NONE;

    if (strcmp(str, "F") == 0 || strcmp(str, "FORWARD") == 0) return CLOUD_CMD_FORWARD;
    if (strcmp(str, "L") == 0 || strcmp(str, "LEFT") == 0)    return CLOUD_CMD_LEFT;
    if (strcmp(str, "R") == 0 || strcmp(str, "RIGHT") == 0)   return CLOUD_CMD_RIGHT;
    if (strcmp(str, "S") == 0 || strcmp(str, "STOP") == 0)    return CLOUD_CMD_STOP;
    if (strcmp(str, "AUTO") == 0)                              return CLOUD_CMD_AUTO;
    if (strcmp(str, "U") == 0 || strcmp(str, "UP") == 0)     return CLOUD_CMD_SURFACE;
    if (strcmp(str, "D") == 0 || strcmp(str, "DOWN") == 0)   return CLOUD_CMD_DIVE;

    return CLOUD_CMD_NONE;
}

esp_err_t cloud_client_init(void)
{
    snprintf(g_upload_url, sizeof(g_upload_url),
             "http://%s:%d/api/upload", CLOUD_SERVER_IP, CLOUD_SERVER_PORT);
    snprintf(g_poll_url, sizeof(g_poll_url),
             "http://%s:%d/api/get_cmd", CLOUD_SERVER_IP, CLOUD_SERVER_PORT);

    ESP_LOGI(TAG, "Cloud client init: upload=%s  poll=%s", g_upload_url, g_poll_url);
    return ESP_OK;
}

static esp_err_t http_post_json(const char *url, const char *json_str)
{
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json_str, strlen(json_str));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        if (status != 200) {
            ESP_LOGW(TAG, "POST status=%d", status);
        }
    }

    esp_http_client_cleanup(client);
    return err;
}

/* 健壮的 HTTP GET: 逐块读取直到结束, 不依赖 Content-Length */
static esp_err_t http_get_string(const char *url, char *buf, size_t buf_size)
{
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        return ESP_FAIL;
    }

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        if (status == 200) {
            int total = 0;
            int n;
            while (total < (int)buf_size - 1) {
                n = esp_http_client_read(client, buf + total, buf_size - total - 1);
                if (n <= 0) break;
                total += n;
            }
            buf[total] = '\0';
        } else {
            err = ESP_FAIL;
        }
    }

    esp_http_client_cleanup(client);
    return err;
}

esp_err_t cloud_upload_sensor(float ph, float roll, float pitch)
{
    char json_buf[128];
    snprintf(json_buf, sizeof(json_buf),
             "{\"do\":2.5,\"ph\":%.2f,\"roll\":%.1f,\"pitch\":%.1f}",
             ph, roll, pitch);

    return http_post_json(g_upload_url, json_buf);
}

esp_err_t cloud_poll_command(cloud_cmd_t *cmd)
{
    if (!cmd) return ESP_ERR_INVALID_ARG;

    *cmd = CLOUD_CMD_NONE;

    char response[256] = {0};
    esp_err_t err = http_get_string(g_poll_url, response, sizeof(response));
    if (err != ESP_OK) {
        return err;
    }

    /* 从 JSON 响应中提取 "cmd":"..." 字段 */
    const char *key = "\"cmd\":\"";
    char *start = strstr(response, key);
    if (!start) {
        return ESP_OK;
    }

    start += strlen(key);
    char *end = strchr(start, '\"');
    if (!end) {
        return ESP_OK;
    }

    size_t len = end - start;
    if (len == 0) {
        return ESP_OK;
    }

    char cmd_str[16];
    size_t copy_len = (len < sizeof(cmd_str) - 1) ? len : sizeof(cmd_str) - 1;
    strncpy(cmd_str, start, copy_len);
    cmd_str[copy_len] = '\0';

    *cmd = parse_command_string(cmd_str);

    if (*cmd != CLOUD_CMD_NONE) {
        ESP_LOGI(TAG, "Received cloud cmd: %s -> %s", cmd_str, cloud_cmd_name(*cmd));
    }

    return ESP_OK;
}