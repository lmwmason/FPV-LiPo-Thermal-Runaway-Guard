#pragma once

typedef struct {
    float soc;
    float v_rc;
    float r0;
    float r1;
    float c1;
} ECMState;

void ecm_init(ECMState *state, float soc_init);
float ecm_predict_voltage(const ECMState *state, float current);
void ecm_update(ECMState *state, float current, float dt);
float ecm_ocv_lookup(float soc);