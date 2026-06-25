#pragma once

#define SIM_MASS                 0.35f
#define SIM_ARM_LENGTH           0.1f
#define SIM_ARM_D                (SIM_ARM_LENGTH * 0.70710678f)
#define SIM_IXX                  2.5e-3f
#define SIM_IYY                  2.5e-3f
#define SIM_IZZ                  4.5e-3f
#define SIM_MAX_THRUST_PER_MOTOR 10.0f
#define SIM_DRAG_COEFF           0.1f
#define SIM_MOTOR_TAU            0.02f
#define SIM_MAX_RATE_ROLL        720.0f
#define SIM_MAX_RATE_PITCH       720.0f
#define SIM_MAX_RATE_YAW         360.0f
#define SIM_GRAVITY              9.81f
#define SIM_YAW_TORQUE_COEFF     0.02f
#define SIM_PI                   3.14159265358979323846f
#define SIM_DEG2RAD              (SIM_PI / 180.0f)
#define SIM_RAD2DEG              (180.0f / SIM_PI)

#define RATE_P_ROLL   0.5f
#define RATE_I_ROLL   0.3f
#define RATE_D_ROLL   0.05f
#define RATE_P_PITCH  0.5f
#define RATE_I_PITCH  0.3f
#define RATE_D_PITCH  0.05f
#define RATE_P_YAW    0.8f
#define RATE_I_YAW    0.4f
#define RATE_D_YAW    0.0f

#define SIM_DT       0.001f
#define SIM_DURATION 20.0f

#define BATTERY_T_AMBIENT          25.0f
#define BATTERY_THERMAL_RESISTANCE 0.5f
#define BATTERY_THERMAL_CAPACITY   15.0f
#define BATTERY_CAPACITY_COULOMB   (2.0f * 3600.0f)
#define BATTERY_VOLTAGE_CUTOFF     2.5f
#define BATTERY_MAX_DERATE         0.15f
