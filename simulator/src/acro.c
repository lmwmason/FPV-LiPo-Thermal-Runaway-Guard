#include "acro.h"
#include "params.h"

#define ACRO_INTEGRAL_LIMIT 3.0f

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void acro_init(AcroController *ctrl) {
    ctrl->roll_integral = 0.0f;
    ctrl->pitch_integral = 0.0f;
    ctrl->yaw_integral = 0.0f;
    ctrl->roll_prev_error = 0.0f;
    ctrl->pitch_prev_error = 0.0f;
    ctrl->yaw_prev_error = 0.0f;
}

static float rate_pid(float sp, float meas, float max_rate_rad,
                       float *integral, float *prev_error,
                       float kp, float ki, float kd, float dt) {
    float err = (sp - meas) / max_rate_rad;
    *integral = clampf(*integral + err * dt, -ACRO_INTEGRAL_LIMIT, ACRO_INTEGRAL_LIMIT);
    float deriv = (err - *prev_error) / dt;
    *prev_error = err;
    return kp * err + ki * (*integral) + kd * deriv;
}

void acro_update(AcroController *ctrl, const RcCommand *rc,
                  float gyro_p, float gyro_q, float gyro_r,
                  float dt, float motor_cmd[4]) {
    float max_roll = SIM_MAX_RATE_ROLL * SIM_DEG2RAD;
    float max_pitch = SIM_MAX_RATE_PITCH * SIM_DEG2RAD;
    float max_yaw = SIM_MAX_RATE_YAW * SIM_DEG2RAD;

    float roll_sp = rc->roll * max_roll;
    float pitch_sp = rc->pitch * max_pitch;
    float yaw_sp = rc->yaw * max_yaw;

    float roll_out = rate_pid(roll_sp, gyro_p, max_roll,
                               &ctrl->roll_integral, &ctrl->roll_prev_error,
                               RATE_P_ROLL, RATE_I_ROLL, RATE_D_ROLL, dt);
    float pitch_out = rate_pid(pitch_sp, gyro_q, max_pitch,
                                &ctrl->pitch_integral, &ctrl->pitch_prev_error,
                                RATE_P_PITCH, RATE_I_PITCH, RATE_D_PITCH, dt);
    float yaw_out = rate_pid(yaw_sp, gyro_r, max_yaw,
                              &ctrl->yaw_integral, &ctrl->yaw_prev_error,
                              RATE_P_YAW, RATE_I_YAW, RATE_D_YAW, dt);

    float base = rc->throttle;
    float m1 = base - roll_out - pitch_out - yaw_out;
    float m2 = base + roll_out + pitch_out - yaw_out;
    float m3 = base + roll_out - pitch_out + yaw_out;
    float m4 = base - roll_out + pitch_out + yaw_out;

    motor_cmd[0] = clampf(m1, 0.0f, 1.0f);
    motor_cmd[1] = clampf(m2, 0.0f, 1.0f);
    motor_cmd[2] = clampf(m3, 0.0f, 1.0f);
    motor_cmd[3] = clampf(m4, 0.0f, 1.0f);
}
