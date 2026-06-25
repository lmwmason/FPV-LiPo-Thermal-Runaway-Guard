#pragma once

typedef struct {
    float soc;
    float v1;
    float v2;
    float r0;
    float r1;
    float c1;
    float r2;
    float c2;
} ECM2State;

void ecm2_init(ECM2State *state, float soc_init);
float ecm2_predict_voltage(const ECM2State *state, float current);
void ecm2_update(ECM2State *state, float current, float dt);
