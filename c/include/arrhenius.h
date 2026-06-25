#pragma once

typedef struct {
    float soh;
    float q_sei;
    float time_to_runaway;
} ArrheniusState;

void arrhenius_init(ArrheniusState *state);
void arrhenius_update(ArrheniusState *state, float soh, float temperature, float dt);
float arrhenius_time_to_runaway(const ArrheniusState *state);