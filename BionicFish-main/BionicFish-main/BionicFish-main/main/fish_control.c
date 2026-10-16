#include "fish_control.h"
#include "fish_config.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "fish_ctrl";

static const char *state_names[] = {
    [FISH_IDLE]       = "IDLE",
    [FISH_LEVEL]      = "LEVEL",
    [FISH_DIVE]       = "DIVE",
    [FISH_SURFACE]    = "SURFACE",
    [FISH_ROLL_LEFT]  = "ROLL LEFT",
    [FISH_ROLL_RIGHT] = "ROLL RIGHT",
    [FISH_EMERGENCY]  = "EMERGENCY"
};

static const char *ext_cmd_names[] = {
    [EXT_CMD_NONE]     = "NONE",
    [EXT_CMD_FORWARD]  = "FORWARD",
    [EXT_CMD_LEFT]     = "LEFT",
    [EXT_CMD_RIGHT]    = "RIGHT",
    [EXT_CMD_STOP]     = "STOP",
    [EXT_CMD_AUTO]     = "AUTO",
    [EXT_CMD_DIVE]     = "DIVE",
    [EXT_CMD_SURFACE]  = "SURFACE",
};

const char *fish_state_name(fish_state_t state)
{
    if (state > FISH_EMERGENCY) return "UNKNOWN";
    return state_names[state];
}

#define LOCK(ctrl)   xSemaphoreTakeRecursive((ctrl)->mutex, portMAX_DELAY)
#define UNLOCK(ctrl) xSemaphoreGiveRecursive((ctrl)->mutex)

static swim_mode_t ext_cmd_to_swim_mode(external_cmd_t cmd)
{
    switch (cmd) {
    case EXT_CMD_FORWARD: return SWIM_FORWARD;
    case EXT_CMD_LEFT:    return SWIM_TURN_LEFT;
    case EXT_CMD_RIGHT:   return SWIM_TURN_RIGHT;
    case EXT_CMD_STOP:    return SWIM_STOP;
    case EXT_CMD_DIVE:    return SWIM_DIVE;
    case EXT_CMD_SURFACE: return SWIM_SURFACE;
    default:              return SWIM_STOP;
    }
}

static swim_mode_t state_to_swim_mode(fish_state_t state)
{
    switch (state) {
    case FISH_LEVEL:      return SWIM_FORWARD;
    case FISH_DIVE:       return SWIM_DIVE;
    case FISH_SURFACE:    return SWIM_SURFACE;
    case FISH_ROLL_LEFT:  return SWIM_TURN_RIGHT;
    case FISH_ROLL_RIGHT: return SWIM_TURN_LEFT;
    case FISH_EMERGENCY:
    case FISH_IDLE:
    default:              return SWIM_STOP;
    }
}

void fish_control_set_external_cmd(fish_control_t *ctrl, external_cmd_t cmd)
{
    LOCK(ctrl);

    if (cmd == EXT_CMD_AUTO) {
        if (ctrl->demo_mode) {
            UNLOCK(ctrl);
            return;
        }
        ctrl->demo_mode = true;
        ctrl->state = FISH_LEVEL;
        ctrl->demo_step = 0;
        ctrl->demo_phase_start = esp_timer_get_time();
        ctrl->compliance_scale = 1.0f;
        ctrl->manual_override = false;
        ctrl->external_cmd = EXT_CMD_NONE;
        ctrl->cmd_source = CMD_SOURCE_NONE;
        for (int i = 0; i < TAIL_JOINT_COUNT; i++) {
            ctrl->servo_estimated[i] = 0.0f;
        }
        fish_motion_set_mode(&ctrl->motion, SWIM_FORWARD);
        ESP_LOGI(TAG, "Demo mode entered via AUTO cmd");
        UNLOCK(ctrl);
        return;
    }

    if (ctrl->cmd_source == CMD_SOURCE_LOCAL && cmd != EXT_CMD_NONE) {
        ESP_LOGI(TAG, "Cloud cmd '%s' ignored (local override active)",
                 ext_cmd_names[cmd]);
        UNLOCK(ctrl);
        return;
    }

    if (ctrl->demo_mode) {
        ctrl->demo_mode = false;
        ESP_LOGI(TAG, "Demo mode exited by external cmd");
    }

    ctrl->external_cmd = cmd;
    ctrl->manual_override = (cmd != EXT_CMD_NONE);
    ctrl->cmd_source = CMD_SOURCE_CLOUD;

    if (cmd != EXT_CMD_NONE) {
        ctrl->compliance_scale = 1.0f;
        ESP_LOGI(TAG, "External cmd: %s", ext_cmd_names[cmd]);
    }

    UNLOCK(ctrl);
}

void fish_control_clear_external_cmd(fish_control_t *ctrl)
{
    LOCK(ctrl);
    ctrl->external_cmd = EXT_CMD_NONE;
    ctrl->manual_override = false;
    ctrl->cmd_source = CMD_SOURCE_NONE;
    UNLOCK(ctrl);
}

void fish_control_set_local_cmd(fish_control_t *ctrl, external_cmd_t cmd)
{
    LOCK(ctrl);

    if (cmd == EXT_CMD_AUTO) {
        if (ctrl->demo_mode) {
            UNLOCK(ctrl);
            return;
        }
        ctrl->demo_mode = true;
        ctrl->state = FISH_LEVEL;
        ctrl->demo_step = 0;
        ctrl->demo_phase_start = esp_timer_get_time();
        ctrl->compliance_scale = 1.0f;
        ctrl->manual_override = false;
        ctrl->external_cmd = EXT_CMD_NONE;
        ctrl->cmd_source = CMD_SOURCE_NONE;
        for (int i = 0; i < TAIL_JOINT_COUNT; i++) {
            ctrl->servo_estimated[i] = 0.0f;
        }
        fish_motion_set_mode(&ctrl->motion, SWIM_FORWARD);
        ESP_LOGI(TAG, "Demo mode entered via local AUTO");
        UNLOCK(ctrl);
        return;
    }

    if (ctrl->demo_mode) {
        ctrl->demo_mode = false;
        ESP_LOGI(TAG, "Demo mode exited by local cmd");
    }

    ctrl->external_cmd = cmd;
    ctrl->manual_override = (cmd != EXT_CMD_NONE);
    ctrl->cmd_source = CMD_SOURCE_LOCAL;

    if (cmd != EXT_CMD_NONE) {
        ctrl->compliance_scale = 1.0f;
        ESP_LOGI(TAG, "Local cmd: %s (override cloud)", ext_cmd_names[cmd]);
    }

    UNLOCK(ctrl);
}

void fish_control_init(fish_control_t *ctrl)
{
    memset(ctrl, 0, sizeof(*ctrl));
    ctrl->mutex = xSemaphoreCreateRecursiveMutex();
    ctrl->state = FISH_IDLE;
    ctrl->prev_state = FISH_IDLE;
    ctrl->pending_state = FISH_IDLE;
    ctrl->debounce_count = 0;
    ctrl->demo_mode = false;
    ctrl->demo_step = 0;
    ctrl->demo_phase_start = 0;
    ctrl->compliance_scale = 1.0f;
    ctrl->last_sensor_read_ms = 0;
    ctrl->external_cmd = EXT_CMD_NONE;
    ctrl->cmd_source = CMD_SOURCE_NONE;
    ctrl->manual_override = false;
    memset(&ctrl->ph_data, 0, sizeof(ctrl->ph_data));
    for (int i = 0; i < TAIL_JOINT_COUNT; i++) {
        ctrl->servo_estimated[i] = 0.0f;
    }
    fish_motion_init(&ctrl->motion);
}

void fish_control_set_demo_mode(fish_control_t *ctrl, bool enable)
{
    LOCK(ctrl);
    ctrl->demo_mode = enable;
    if (enable) {
        ctrl->state = FISH_LEVEL;
        ctrl->demo_step = 0;
        ctrl->demo_phase_start = esp_timer_get_time();
        ctrl->compliance_scale = 1.0f;
        ctrl->manual_override = false;
        ctrl->external_cmd = EXT_CMD_NONE;
        ctrl->cmd_source = CMD_SOURCE_NONE;
        for (int i = 0; i < TAIL_JOINT_COUNT; i++) {
            ctrl->servo_estimated[i] = 0.0f;
        }
        fish_motion_set_mode(&ctrl->motion, SWIM_FORWARD);
    }
    UNLOCK(ctrl);
}

static void servo_track_update(fish_control_t *ctrl, float dt);

static const uint32_t demo_durations[] = { 3000000, 2000000, 3000000, 2000000 };
static const swim_mode_t demo_modes[] = { SWIM_FORWARD, SWIM_TURN_LEFT, SWIM_FORWARD, SWIM_TURN_RIGHT };
#define DEMO_STEPS 4

static void demo_update(fish_control_t *ctrl)
{
    uint32_t elapsed = esp_timer_get_time() - ctrl->demo_phase_start;

    if (elapsed >= demo_durations[ctrl->demo_step]) {
        ctrl->demo_step = (ctrl->demo_step + 1) % DEMO_STEPS;
        ctrl->demo_phase_start = esp_timer_get_time();
        fish_motion_set_mode(&ctrl->motion, demo_modes[ctrl->demo_step]);
    }

    float dt = 1.0f / CONTROL_LOOP_FREQ_HZ;
    fish_motion_update(&ctrl->motion, dt);
    fish_motion_get_angles(&ctrl->motion, ctrl->joint_angles);

    for (int i = 0; i < TAIL_JOINT_COUNT; i++) {
        ctrl->joint_angles[i] *= ctrl->compliance_scale;
    }
    servo_track_update(ctrl, dt);

    uint32_t now_ms = esp_timer_get_time() / 1000;
    if (now_ms - ctrl->last_sensor_read_ms >= PH_READ_PERIOD_MS) {
        if (bmi270_read_sensor(&ctrl->sensor) == ESP_OK) {
            bmi270_calculate_attitude(&ctrl->sensor, &ctrl->attitude);
        }
        ctrl->last_sensor_read_ms = now_ms;
    }
}

static void servo_track_update(fish_control_t *ctrl, float dt)
{
    float max_step = SERVO_MAX_SPEED_DEG_S * dt;
    float max_lag = 0.0f;

    for (int i = 0; i < TAIL_JOINT_COUNT; i++) {
        float target = ctrl->joint_angles[i];
        float current = ctrl->servo_estimated[i];
        float error = target - current;

        float lag = fabsf(error);
        if (lag > max_lag) max_lag = lag;

        if (error > max_step) {
            current += max_step;
        } else if (error < -max_step) {
            current -= max_step;
        } else {
            current = target;
        }

        ctrl->servo_estimated[i] = current;
        ctrl->joint_angles[i] = current;
    }

    if (max_lag > SERVO_LAG_THRESHOLD_DEG) {
        ctrl->compliance_scale -= 0.005f;
    } else {
        ctrl->compliance_scale += 0.002f;
    }
    if (ctrl->compliance_scale > 1.0f) ctrl->compliance_scale = 1.0f;
    if (ctrl->compliance_scale < SERVO_COMPLIANCE_MIN) ctrl->compliance_scale = SERVO_COMPLIANCE_MIN;
}

static fish_state_t determine_state(fish_state_t current_state,
                                    float pitch, float roll)
{
    if (current_state == FISH_DIVE) {
        if (pitch < PITCH_DIVE_THRESHOLD - HYSTERESIS_MARGIN) {
            goto check_other;
        }
        return FISH_DIVE;
    }

    if (current_state == FISH_SURFACE) {
        if (pitch > PITCH_SURFACE_THRESHOLD + HYSTERESIS_MARGIN) {
            goto check_other;
        }
        return FISH_SURFACE;
    }

    if (current_state == FISH_ROLL_LEFT) {
        if (roll < ROLL_LEFT_THRESHOLD - HYSTERESIS_MARGIN) {
            goto check_other;
        }
        return FISH_ROLL_LEFT;
    }

    if (current_state == FISH_ROLL_RIGHT) {
        if (roll > ROLL_RIGHT_THRESHOLD + HYSTERESIS_MARGIN) {
            goto check_other;
        }
        return FISH_ROLL_RIGHT;
    }

check_other:
    if (pitch > PITCH_DIVE_THRESHOLD) {
        return FISH_DIVE;
    } else if (pitch < PITCH_SURFACE_THRESHOLD) {
        return FISH_SURFACE;
    } else if (roll > ROLL_LEFT_THRESHOLD) {
        return FISH_ROLL_LEFT;
    } else if (roll < ROLL_RIGHT_THRESHOLD) {
        return FISH_ROLL_RIGHT;
    }

    return FISH_LEVEL;
}

void fish_control_update(fish_control_t *ctrl)
{
    LOCK(ctrl);

    if (ctrl->demo_mode) {
        demo_update(ctrl);
        UNLOCK(ctrl);
        return;
    }

    /* 云端指令检查：本地指令优先，本地有效时忽略云端指令 */
    if (ctrl->manual_override && ctrl->cmd_source == CMD_SOURCE_CLOUD) {
        fish_motion_set_mode(&ctrl->motion, ext_cmd_to_swim_mode(ctrl->external_cmd));
        float dt = 1.0f / CONTROL_LOOP_FREQ_HZ;
        fish_motion_update(&ctrl->motion, dt);
        fish_motion_get_angles(&ctrl->motion, ctrl->joint_angles);
        for (int i = 0; i < TAIL_JOINT_COUNT; i++) {
            ctrl->joint_angles[i] *= ctrl->compliance_scale;
        }
        servo_track_update(ctrl, dt);
        UNLOCK(ctrl);
        return;
    }

    /* 本地指令或手动覆盖模式 */
    if (ctrl->manual_override) {
        fish_motion_set_mode(&ctrl->motion, ext_cmd_to_swim_mode(ctrl->external_cmd));
        float dt = 1.0f / CONTROL_LOOP_FREQ_HZ;
        fish_motion_update(&ctrl->motion, dt);
        fish_motion_get_angles(&ctrl->motion, ctrl->joint_angles);
        for (int i = 0; i < TAIL_JOINT_COUNT; i++) {
            ctrl->joint_angles[i] *= ctrl->compliance_scale;
        }
        servo_track_update(ctrl, dt);
        UNLOCK(ctrl);
        return;
    }

    /* ========== 姿态自动控制模式 ========== */
    esp_err_t ret = bmi270_read_sensor(&ctrl->sensor);
    if (ret != ESP_OK) {
        ctrl->prev_state = ctrl->state;
        ctrl->state = FISH_EMERGENCY;
        ctrl->pending_state = FISH_EMERGENCY;
        ctrl->debounce_count = 0;
        fish_motion_set_mode(&ctrl->motion, SWIM_STOP);
        fish_motion_update(&ctrl->motion, 1.0f / CONTROL_LOOP_FREQ_HZ);
        fish_motion_get_angles(&ctrl->motion, ctrl->joint_angles);
        servo_track_update(ctrl, 1.0f / CONTROL_LOOP_FREQ_HZ);
        UNLOCK(ctrl);
        return;
    }

    bmi270_calculate_attitude(&ctrl->sensor, &ctrl->attitude);

    float pitch = ctrl->attitude.pitch;
    float roll  = ctrl->attitude.roll;

    if (fabsf(pitch) < ATTITUDE_DEAD_ZONE) pitch = 0.0f;
    if (fabsf(roll)  < ATTITUDE_DEAD_ZONE) roll  = 0.0f;

    fish_state_t raw_state = determine_state(ctrl->state, pitch, roll);

    if (raw_state != ctrl->pending_state) {
        ctrl->pending_state = raw_state;
        ctrl->debounce_count = 0;
    } else {
        ctrl->debounce_count++;
    }

    if (ctrl->debounce_count >= STATE_DEBOUNCE_COUNT &&
        ctrl->pending_state != ctrl->state) {

        ctrl->prev_state = ctrl->state;
        ctrl->state = ctrl->pending_state;
        ctrl->state_enter_time = esp_timer_get_time();
        ctrl->compliance_scale = 1.0f;

        ESP_LOGI(TAG, "State: %s → %s (pitch=%.1f roll=%.1f)",
                 fish_state_name(ctrl->prev_state),
                 fish_state_name(ctrl->state),
                 ctrl->attitude.pitch, ctrl->attitude.roll);

        fish_motion_set_mode(&ctrl->motion, state_to_swim_mode(ctrl->state));
    }

    float dt = 1.0f / CONTROL_LOOP_FREQ_HZ;
    fish_motion_update(&ctrl->motion, dt);
    fish_motion_get_angles(&ctrl->motion, ctrl->joint_angles);

    for (int i = 0; i < TAIL_JOINT_COUNT; i++) {
        ctrl->joint_angles[i] *= ctrl->compliance_scale;
    }
    servo_track_update(ctrl, dt);

    UNLOCK(ctrl);
}

void fish_control_set_ph_data(fish_control_t *ctrl, const ph_data_t *data)
{
    if (!ctrl || !data) return;
    LOCK(ctrl);
    memcpy(&ctrl->ph_data, data, sizeof(ph_data_t));
    UNLOCK(ctrl);
}

void fish_control_get_snapshot(fish_control_t *ctrl, fish_snapshot_t *snap)
{
    if (!ctrl || !snap) return;
    LOCK(ctrl);
    memcpy(&snap->ph_data, &ctrl->ph_data, sizeof(ph_data_t));
    snap->attitude = ctrl->attitude;
    snap->state = ctrl->state;
    UNLOCK(ctrl);
}