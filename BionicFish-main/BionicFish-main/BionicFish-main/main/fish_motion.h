#ifndef FISH_MOTION_H
#define FISH_MOTION_H

#include <stdint.h>
#include "fish_config.h"

typedef enum {
    SWIM_STOP = 0,
    SWIM_FORWARD,
    SWIM_BACKWARD,
    SWIM_TURN_LEFT,
    SWIM_TURN_RIGHT,
    SWIM_DIVE,
    SWIM_SURFACE
} swim_mode_t;

typedef struct {
    float amplitude;
    float phase;
    float output;
} cpg_oscillator_t;

typedef struct {
    cpg_oscillator_t joints[TAIL_JOINT_COUNT];
    swim_mode_t mode;
    float speed_factor;
    float target_speed_factor;
    float turn_offset;
    float target_turn_offset;
    float global_phase;
    float target_amplitudes[TAIL_JOINT_COUNT];
    float phase_sign;
    float target_phase_sign;
    float target_phases[TAIL_JOINT_COUNT];
} fish_motion_t;

void fish_motion_init(fish_motion_t *motion);
void fish_motion_set_mode(fish_motion_t *motion, swim_mode_t mode);
void fish_motion_set_speed(fish_motion_t *motion, float factor);
void fish_motion_update(fish_motion_t *motion, float dt);
void fish_motion_get_angles(fish_motion_t *motion, float angles[TAIL_JOINT_COUNT]);
const char *fish_motion_mode_name(swim_mode_t mode);

#endif