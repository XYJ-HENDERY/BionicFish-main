#ifndef FISH_CONTROL_H
#define FISH_CONTROL_H

#include "bmi270_driver.h"
#include "fish_motion.h"
#include "ph_sensor.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef enum {
    FISH_IDLE = 0,
    FISH_LEVEL,
    FISH_DIVE,
    FISH_SURFACE,
    FISH_ROLL_LEFT,
    FISH_ROLL_RIGHT,
    FISH_EMERGENCY
} fish_state_t;

/* 外部云指令映射（与 cloud_client.h 对应） */
typedef enum {
    EXT_CMD_NONE = 0,
    EXT_CMD_FORWARD,
    EXT_CMD_LEFT,
    EXT_CMD_RIGHT,
    EXT_CMD_STOP,
    EXT_CMD_AUTO,
    EXT_CMD_DIVE,
    EXT_CMD_SURFACE
} external_cmd_t;

/* 指令来源：本地优先于云端 */
typedef enum {
    CMD_SOURCE_NONE = 0,
    CMD_SOURCE_CLOUD,
    CMD_SOURCE_LOCAL,
} cmd_source_t;

/* 线程安全的传感器数据快照 */
typedef struct {
    ph_data_t    ph_data;
    attitude_t   attitude;
    fish_state_t state;
} fish_snapshot_t;

typedef struct {
    SemaphoreHandle_t mutex;
    bmi270_data_t  sensor;
    attitude_t     attitude;
    fish_state_t   state;
    fish_state_t   prev_state;
    fish_state_t   pending_state;
    uint32_t       debounce_count;
    fish_motion_t  motion;
    float          joint_angles[TAIL_JOINT_COUNT];
    float          servo_estimated[TAIL_JOINT_COUNT];
    float          compliance_scale;
    uint32_t       state_enter_time;
    bool           demo_mode;
    uint32_t       demo_phase_start;
    int            demo_step;
    uint32_t       last_sensor_read_ms;
    ph_data_t      ph_data;
    /* 外部云指令 */
    external_cmd_t external_cmd;
    cmd_source_t   cmd_source;
    bool           manual_override;
} fish_control_t;

void fish_control_init(fish_control_t *ctrl);
void fish_control_set_demo_mode(fish_control_t *ctrl, bool enable);
void fish_control_update(fish_control_t *ctrl);
void fish_control_set_external_cmd(fish_control_t *ctrl, external_cmd_t cmd);
void fish_control_set_local_cmd(fish_control_t *ctrl, external_cmd_t cmd);
void fish_control_clear_external_cmd(fish_control_t *ctrl);
const char *fish_state_name(fish_state_t state);

void fish_control_set_ph_data(fish_control_t *ctrl, const ph_data_t *data);
void fish_control_get_snapshot(fish_control_t *ctrl, fish_snapshot_t *snap);

#endif