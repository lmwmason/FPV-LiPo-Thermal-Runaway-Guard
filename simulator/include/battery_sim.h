#pragma once

#include "ecm.h"
#include "ekf.h"
#include "arrhenius.h"
#include "controller.h"

typedef struct {
    float current;
    float voltage;
    float soc;
    float soh;
    float r0;
    float temperature;
    float q_sei;
    float time_to_runaway;
    float output_limit;
    int status;
} BatterySimOutput;

typedef struct {
    ECMState true_ecm;
    EKFState ekf;
    ArrheniusState arrhenius;
    ControllerState controller;
    float soh;
    float temperature;
} BatterySim;

void battery_sim_init(BatterySim *sim, float soh_init);
void battery_sim_step(BatterySim *sim, const float motor_thrust[4], float dt, BatterySimOutput *out);
