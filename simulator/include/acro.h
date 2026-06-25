#pragma once

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float throttle;
} RcCommand;

typedef struct {
    float roll_integral;
    float pitch_integral;
    float yaw_integral;
    float roll_prev_error;
    float pitch_prev_error;
    float yaw_prev_error;
} AcroController;

void acro_init(AcroController *ctrl);
void acro_update(AcroController *ctrl, const RcCommand *rc,
                  float gyro_p, float gyro_q, float gyro_r,
                  float dt, float motor_cmd[4]);
