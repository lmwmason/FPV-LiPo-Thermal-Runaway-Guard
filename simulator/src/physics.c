#include "physics.h"
#include "params.h"
#include <math.h>

void quad_init(QuadState *state) {
    state->pos.x = 0.0f;
    state->pos.y = 0.0f;
    state->pos.z = 0.0f;
    state->vel.x = 0.0f;
    state->vel.y = 0.0f;
    state->vel.z = 0.0f;
    state->quat.w = 1.0f;
    state->quat.x = 0.0f;
    state->quat.y = 0.0f;
    state->quat.z = 0.0f;
    state->omega.x = 0.0f;
    state->omega.y = 0.0f;
    state->omega.z = 0.0f;
    state->motor_thrust[0] = 0.0f;
    state->motor_thrust[1] = 0.0f;
    state->motor_thrust[2] = 0.0f;
    state->motor_thrust[3] = 0.0f;
}

static void compute_derivative(const float s[13], const float thrust[4], float ds[13]) {
    float vx = s[3], vy = s[4], vz = s[5];
    float qw = s[6], qx = s[7], qy = s[8], qz = s[9];
    float p = s[10], q = s[11], r = s[12];

    float T1 = thrust[0], T2 = thrust[1], T3 = thrust[2], T4 = thrust[3];
    float fz_body = T1 + T2 + T3 + T4;
    float tau_x = SIM_ARM_D * (T2 + T3 - T1 - T4);
    float tau_y = SIM_ARM_D * (T2 + T4 - T1 - T3);
    float tau_z = SIM_YAW_TORQUE_COEFF * (T3 + T4 - T1 - T2);

    float fx_world = 2.0f * fz_body * (qw * qy + qx * qz);
    float fy_world = 2.0f * fz_body * (qy * qz - qw * qx);
    float fz_world = fz_body * (1.0f - 2.0f * qx * qx - 2.0f * qy * qy);

    ds[0] = vx;
    ds[1] = vy;
    ds[2] = vz;
    ds[3] = (fx_world - SIM_DRAG_COEFF * vx * fabsf(vx)) / SIM_MASS;
    ds[4] = (fy_world - SIM_DRAG_COEFF * vy * fabsf(vy)) / SIM_MASS;
    ds[5] = (fz_world - SIM_MASS * SIM_GRAVITY - SIM_DRAG_COEFF * vz * fabsf(vz)) / SIM_MASS;

    ds[6] = 0.5f * (-qx * p - qy * q - qz * r);
    ds[7] = 0.5f * (qw * p + qy * r - qz * q);
    ds[8] = 0.5f * (qw * q - qx * r + qz * p);
    ds[9] = 0.5f * (qw * r + qx * q - qy * p);

    ds[10] = (tau_x + (SIM_IYY - SIM_IZZ) * q * r) / SIM_IXX;
    ds[11] = (tau_y + (SIM_IZZ - SIM_IXX) * r * p) / SIM_IYY;
    ds[12] = (tau_z + (SIM_IXX - SIM_IYY) * p * q) / SIM_IZZ;
}

void quad_step(QuadState *state, const float motor_cmd[4], float dt) {
    for (int i = 0; i < 4; i++) {
        float target = motor_cmd[i] * SIM_MAX_THRUST_PER_MOTOR;
        float alpha = 1.0f - expf(-dt / SIM_MOTOR_TAU);
        state->motor_thrust[i] += (target - state->motor_thrust[i]) * alpha;
    }

    float s[13] = {
        state->pos.x, state->pos.y, state->pos.z,
        state->vel.x, state->vel.y, state->vel.z,
        state->quat.w, state->quat.x, state->quat.y, state->quat.z,
        state->omega.x, state->omega.y, state->omega.z
    };

    float k1[13], k2[13], k3[13], k4[13], tmp[13];

    compute_derivative(s, state->motor_thrust, k1);
    for (int i = 0; i < 13; i++) tmp[i] = s[i] + 0.5f * dt * k1[i];

    compute_derivative(tmp, state->motor_thrust, k2);
    for (int i = 0; i < 13; i++) tmp[i] = s[i] + 0.5f * dt * k2[i];

    compute_derivative(tmp, state->motor_thrust, k3);
    for (int i = 0; i < 13; i++) tmp[i] = s[i] + dt * k3[i];

    compute_derivative(tmp, state->motor_thrust, k4);

    for (int i = 0; i < 13; i++)
        s[i] += (dt / 6.0f) * (k1[i] + 2.0f * k2[i] + 2.0f * k3[i] + k4[i]);

    float qnorm = sqrtf(s[6] * s[6] + s[7] * s[7] + s[8] * s[8] + s[9] * s[9]);
    s[6] /= qnorm;
    s[7] /= qnorm;
    s[8] /= qnorm;
    s[9] /= qnorm;

    state->pos.x = s[0]; state->pos.y = s[1]; state->pos.z = s[2];
    state->vel.x = s[3]; state->vel.y = s[4]; state->vel.z = s[5];
    state->quat.w = s[6]; state->quat.x = s[7]; state->quat.y = s[8]; state->quat.z = s[9];
    state->omega.x = s[10]; state->omega.y = s[11]; state->omega.z = s[12];
}

void quad_get_euler(const QuadState *state, float *roll, float *pitch, float *yaw) {
    float w = state->quat.w, x = state->quat.x, y = state->quat.y, z = state->quat.z;
    *roll = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    float sp = 2.0f * (w * y - z * x);
    sp = fmaxf(-1.0f, fminf(1.0f, sp));
    *pitch = asinf(sp);
    *yaw = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}
