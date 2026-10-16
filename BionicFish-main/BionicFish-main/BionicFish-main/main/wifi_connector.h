#ifndef WIFI_CONNECTOR_H
#define WIFI_CONNECTOR_H

#include "esp_err.h"
#include <stdbool.h>

esp_err_t wifi_connector_init(void);
bool wifi_connector_is_connected(void);

#endif