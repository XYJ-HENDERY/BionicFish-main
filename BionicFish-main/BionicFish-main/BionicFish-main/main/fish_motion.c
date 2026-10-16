#include "fish_motion.h"
#include "fish_config.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define TWO_PI (2.0f * (float)M_PI)

static const float base_amplitudes[TAIL_JOINT_COUNT] = {
    AMP_JOINT_0,
    AMP_JOINT_1,
    AMP_JOINT_2,
};

static const char *mode_names[] = {
    [SWIM_STOP]       = "STOP",
    [SWIM_FORWARD]    = "FORWARD",
    [SWIM_BACKWARD]   = "BACKWARD",
    [SWIM_TURN_LEFT]  = "TURN LEFT",
    [SWIM_TURN_RIGHT] = "TURN RIGHT",
    [SWIM_DIVE]       = "DIVE",
    [SWIM_SURFACE]    = "SURFACE",
};

const char *fish_motion_mode_name(swim_mode_t mode)
{
    if (mode > SWIM_SURFACE) return "UNKNOWN";
    return mode_names[mode];
}

static float lerpf(float a, float b, float t)
{
    return a + (b - a) * t;
}

static float clampf(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

static void motion_apply_params(fish_motion_t *motion, const float *amps,
                                 float phase_sign, float offset)
{
    for (int i = 0; i < TAIL_JOINT_COUNT; i++) {
        motion->target_amplitudes[i] = amps[i];
        motion->target_phases[i] = phase_sign * i * PHASE_LAG_PER_JOINT;
    }
    motion->target_phase_sign = phase_sign;
    motion->target_turn_offset = offset;
}

void fish_motion_init(fish_motion_t *motion)
{
    memset(motion, 0, sizeof(*motion));
    fish_motion_set_mode(motion, SWIM_STOP);
    motion->speed_factor = 1.0f;
    motion->target_speed_factor = 1.0f;
    motion->turn_offset = 0.0f;
    motion->target_turn_offset = 0.0f;
    motion->phase_sign = -1.0f;
    motion->target_phase_sign = -1.0f;
    for (int i = 0; i < TAIL_JOINT_COUNT; i++) {
        motion->joints[i].phase = motion->phase_sign * i * PHASE_LAG_PER_JOINT;
        motion->target_phases[i] = motion->joints[i].phase;
    }
}

void fish_motion_set_mode(fish_motion_t *motion, swim_mode_t mode)
{
    if (mode == motion->mode) return;

    motion->mode = mode;

    switch (mode) {
    case SWIM_STOP:
        motion_apply_params(motion,
            (float[]){0.0f, 0.0f, 0.0f}, -1.0f, 0.0f);
        break;

    case SWIM_FORWARD:
        motion_apply_params(motion, base_amplitudes, -1.0f, 0.0f);
        break;

    case SWIM_BACKWARD:
        motion_apply_params(motion,
            (float[]){AMP_JOINT_0 * 0.7f, AMP_JOINT_1 * 0.7f, AMP_JOINT_2 * 0.7f},
            +1.0f, 0.0f);
        break;

    case SWIM_TURN_LEFT:
        motion_apply_params(motion, base_amplitudes, -1.0f, TURN_OFFSET_DEG);
        break;

    case SWIM_TURN_RIGHT:
        motion_apply_params(motion, base_amplitudes, -1.0f, -TURN_OFFSET_DEG);
        break;

    case SWIM_DIVE:
        motion_apply_params(motion,
            (float[]){AMP_JOINT_0 * 1.0f, AMP_JOINT_1 * 0.85f, AMP_JOINT_2 * 0.7f},
            -1.0f, 0.0f);
        break;

    case SWIM_SURFACE:
        motion_apply_params(motion,
            (float[]){AMP_JOINT_0 * 0.7f, AMP_JOINT_1 * 0.85f, AMP_JOINT_2 * 1.0f},
            -1.0f, 0.0f);
        break;
    }
}

void fish_motion_set_speed(fish_motion_t *motion, float factor)
{
    if (factor < 0.1f) factor = 0.1f;
    if (factor > 2.5f) factor = 2.5f;
    motion->target_speed_factor = factor;
}

void fish_motion_update(fish_motion_t *motion, float dt)
{
    float blend = clampf(MOTION_TRANSITION_SPEED * dt, 0.0f, 1.0f);

    for (int i = 0; i < TAIL_JOINT_COUNT; i++) {
        motion->joints[i].amplitude = lerpf(motion->joints[i].amplitude,
                                            motion->target_amplitudes[i], blend);
        motion->joints[i].phase = lerpf(motion->joints[i].phase,
                                        motion->target_phases[i], blend);
    }

    motion->phase_sign = lerpf(motion->phase_sign,
                               motion->target_phase_sign, blend);

    float turn_blend = clampf(TURN_TRANSITION_SPEED * dt, 0.0f, 1.0f);
    motion->turn_offset = lerpf(motion->turn_offset,
                                motion->target_turn_offset, turn_blend);

    float speed_blend = clampf(SPEED_RAMP_RATE * dt, 0.0f, 1.0f);
    motion->speed_factor = lerpf(motion->speed_factor,
                                 motion->target_speed_factor, speed_blend);

    float omega = TWO_PI * CPG_FREQ_CRUISE * motion->speed_factor;
    motion->global_phase = fmodf(motion->global_phase + omega * dt, TWO_PI);

    for (int i = 0; i < TAIL_JOINT_COUNT; i++) {
        motion->joints[i].output =
            motion->joints[i].amplitude *
            sinf(motion->global_phase + motion->joints[i].phase)
            + motion->turn_offset;
    }
}

void fish_motion_get_angles(fish_motion_t *motion, float angles[TAIL_JOINT_COUNT])
{
    for (int i = 0; i < TAIL_JOINT_COUNT; i++) {
        angles[i] = clampf(motion->joints[i].output,
                          -SERVO_MAX_ANGLE_DEG, SERVO_MAX_ANGLE_DEG);
    }
}