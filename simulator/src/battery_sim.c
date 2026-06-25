#include "battery_sim.h"
#include "params.h"
#include "ecm_params.h"
#include <math.h>

void battery_sim_init(BatterySim *sim, float soh_init) {
    ecm_init(&sim->true_ecm, 1.0f);
    sim->true_ecm.r0 = ECM_R0_INIT / soh_init;
    ekf_init(&sim->ekf, &sim->true_ecm);
    arrhenius_init(&sim->arrhenius);
    controller_init(&sim->controller);
    sim->soh = soh_init;
    sim->temperature = BATTERY_T_AMBIENT;
}

void battery_sim_step(BatterySim *sim, const float motor_thrust[4], float dt, BatterySimOutput *out) {
    float m1 = motor_thrust[0] / SIM_MAX_THRUST_PER_MOTOR;
    float m2 = motor_thrust[1] / SIM_MAX_THRUST_PER_MOTOR;
    float m3 = motor_thrust[2] / SIM_MAX_THRUST_PER_MOTOR;
    float m4 = motor_thrust[3] / SIM_MAX_THRUST_PER_MOTOR;
    float throttle_total = (m1 + m2 + m3 + m4) / 4.0f;

    float derate = fminf(1.0f / sim->soh - 1.0f, BATTERY_MAX_DERATE);
    float current = throttle_total * throttle_total * 40.0f * (1.0f - derate);

    ecm_update(&sim->true_ecm, current, dt);
    sim->true_ecm.soc -= fabsf(current) * dt / BATTERY_CAPACITY_COULOMB;
    sim->true_ecm.soc = fmaxf(0.0f, fminf(1.0f, sim->true_ecm.soc));

    float voltage = ecm_predict_voltage(&sim->true_ecm, current);
    voltage = fmaxf(voltage, BATTERY_VOLTAGE_CUTOFF);

    float p_heat = current * current * sim->true_ecm.r0;
    float dT = (p_heat - (sim->temperature - BATTERY_T_AMBIENT) / BATTERY_THERMAL_RESISTANCE)
               * dt / BATTERY_THERMAL_CAPACITY;
    sim->temperature += dT;

    ekf_update(&sim->ekf, voltage, current, dt);
    arrhenius_update(&sim->arrhenius, sim->soh, sim->temperature, dt);
    controller_update(&sim->controller, &sim->arrhenius, sim->soh);

    out->current = current;
    out->voltage = voltage;
    out->soc = sim->ekf.x[0];
    out->soh = sim->soh;
    out->r0 = sim->ekf.x[2];
    out->temperature = sim->temperature;
    out->q_sei = sim->arrhenius.q_sei;
    out->time_to_runaway = sim->arrhenius.time_to_runaway;
    out->output_limit = sim->controller.output_limit;
    out->status = (int)sim->controller.status;
}
