#ifndef LOCAL_SERVER_H
#define LOCAL_SERVER_H

#include "esp_err.h"
#include "fish_control.h"

void local_server_set_control(fish_control_t *ctrl);
esp_err_t local_server_start(void);
void local_server_stop(void);

#endif