#ifndef CLOUD_CLIENT_H
#define CLOUD_CLIENT_H

#include "esp_err.h"
#include "fish_control.h"

typedef enum {
    CLOUD_CMD_NONE = 0,
    CLOUD_CMD_FORWARD,
    CLOUD_CMD_LEFT,
    CLOUD_CMD_RIGHT,
    CLOUD_CMD_STOP,
    CLOUD_CMD_AUTO,
    CLOUD_CMD_DIVE,
    CLOUD_CMD_SURFACE
} cloud_cmd_t;

esp_err_t cloud_client_init(void);
esp_err_t cloud_upload_sensor(float ph, float roll, float pitch);
esp_err_t cloud_poll_command(cloud_cmd_t *cmd);
const char *cloud_cmd_name(cloud_cmd_t cmd);

#endif