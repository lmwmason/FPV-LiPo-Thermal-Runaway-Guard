#include "arrhenius.h"
#include <math.h>

#define ARRHENIUS_A      880.0f
#define ARRHENIUS_EA     50000.0f
#define ARRHENIUS_R      8.314f
#define Q_SEI_THRESHOLD  10.0f

void arrhenius_init(ArrheniusState *state) {
    state->soh = 1.0f;
    state->q_sei = 0.0f;
    state->time_to_runaway = 1e9f;
}

void arrhenius_update(ArrheniusState *state, float soh, float temperature, float dt) {
    state->soh = soh;

    float T_kelvin = temperature + 273.15f;
    float k = ARRHENIUS_A * expf(-ARRHENIUS_EA / (ARRHENIUS_R * T_kelvin));
    float soh_factor = 1.0f - soh;
    float k_current = k * (1.0f + soh_factor * 5.0f);
    float dq = k_current * dt;
    state->q_sei += dq;

    float q_remaining = Q_SEI_THRESHOLD - state->q_sei;
    if (q_remaining <= 0.0f) {
        state->time_to_runaway = 0.0f;
    } else if (k_current > 0.0f) {
        state->time_to_runaway = q_remaining / k_current;
    } else {
        state->time_to_runaway = 1e9f;
    }
}

float arrhenius_time_to_runaway(const ArrheniusState *state) {
    return state->time_to_runaway;
}