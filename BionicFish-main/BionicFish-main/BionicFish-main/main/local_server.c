#include "local_server.h"
#include "fish_config.h"
#include "fish_control.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "http_srv";

static httpd_handle_t s_server = NULL;
static fish_control_t *s_ctrl = NULL;

void local_server_set_control(fish_control_t *ctrl)
{
    s_ctrl = ctrl;
}

/* GET /action?go=X  处理方向键命令 */
static esp_err_t action_handler(httpd_req_t *req)
{
    char query[64] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing query");
        return ESP_FAIL;
    }

    char go[16] = {0};
    if (httpd_query_key_value(query, "go", go, sizeof(go)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing go param");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Local cmd: go=%s", go);

    if (s_ctrl) {
        if (strcmp(go, "S") == 0) {
            /* 停止 = 清除手动覆盖, 回到姿态自动控制 */
            fish_control_clear_external_cmd(s_ctrl);
        } else {
            external_cmd_t cmd = EXT_CMD_NONE;
            if      (strcmp(go, "F") == 0)    cmd = EXT_CMD_FORWARD;
            else if (strcmp(go, "L") == 0)    cmd = EXT_CMD_LEFT;
            else if (strcmp(go, "R") == 0)    cmd = EXT_CMD_RIGHT;
            else if (strcmp(go, "U") == 0)    cmd = EXT_CMD_SURFACE;
            else if (strcmp(go, "D") == 0)    cmd = EXT_CMD_DIVE;
            else if (strcmp(go, "AUTO") == 0) cmd = EXT_CMD_AUTO;
            if (cmd != EXT_CMD_NONE) {
                fish_control_set_local_cmd(s_ctrl, cmd);
            }
        }
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_sendstr(req, "ok");
    return ESP_OK;
}

/* GET /api/status  返回当前传感器数据（JSON） */
static esp_err_t status_handler(httpd_req_t *req)
{
    char buf[256];
    if (s_ctrl) {
        fish_snapshot_t snap;
        fish_control_get_snapshot(s_ctrl, &snap);

        snprintf(buf, sizeof(buf),
                 "{\"do\":2.5,\"ph\":%.2f,\"roll\":%.1f,\"pitch\":%.1f,"
                 "\"state\":\"%s\"}",
                 snap.ph_data.ph_value,
                 snap.attitude.roll,
                 snap.attitude.pitch,
                 fish_state_name(snap.state));
    } else {
        snprintf(buf, sizeof(buf), "{}");
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

esp_err_t local_server_start(void)
{
    if (s_server) {
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = LOCAL_WEB_SERVER_PORT;
    config.max_uri_handlers = 8;
    config.stack_size = 4096;

    if (httpd_start(&s_server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Server start failed");
        return ESP_FAIL;
    }

    httpd_uri_t action_uri = {
        .uri       = "/action",
        .method    = HTTP_GET,
        .handler   = action_handler,
        .user_ctx  = NULL,
    };
    httpd_register_uri_handler(s_server, &action_uri);

    httpd_uri_t status_uri = {
        .uri       = "/api/status",
        .method    = HTTP_GET,
        .handler   = status_handler,
        .user_ctx  = NULL,
    };
    httpd_register_uri_handler(s_server, &status_uri);

    ESP_LOGI(TAG, "Local HTTP server started on port %d", LOCAL_WEB_SERVER_PORT);
    return ESP_OK;
}

void local_server_stop(void)
{
    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
    }
}