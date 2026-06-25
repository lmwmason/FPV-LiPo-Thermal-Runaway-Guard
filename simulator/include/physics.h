#pragma once

typedef struct {
    float x, y, z;
} Vec3;

typedef struct {
    float w, x, y, z;
} Quat;

typedef struct {
    Vec3 pos;
    Vec3 vel;
    Quat quat;
    Vec3 omega;
    float motor_thrust[4];
} QuadState;

void quad_init(QuadState *state);
void quad_step(QuadState *state, const float motor_cmd[4], float dt);
void quad_get_euler(const QuadState *state, float *roll, float *pitch, float *yaw);
