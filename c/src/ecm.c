#include "ecm.h"
#include "ocv_table.h"
#include "ecm_params.h"
#include <math.h>

void ecm_init(ECMState *state, float soc_init) {
    state->soc = soc_init;
    state->v_rc = 0.0f;
    state->r0 = ECM_R0_INIT;
    state->r1 = ECM_R1_INIT;
    state->c1 = ECM_C1_INIT;
}

float ecm_ocv_lookup(float soc) {
    if (soc <= 0.0f) return OCV_SOC_TABLE[0];
    if (soc >= 1.0f) return OCV_SOC_TABLE[OCV_TABLE_SIZE - 1];

    float idx = soc * (OCV_TABLE_SIZE - 1);
    int i = (int)idx;
    float t = idx - i;
    return OCV_SOC_TABLE[i] * (1.0f - t) + OCV_SOC_TABLE[i + 1] * t;
}

float ecm_predict_voltage(const ECMState *state, float current) {
    float ocv = ecm_ocv_lookup(state->soc);
    return ocv - fabsf(current) * state->r0 - state->v_rc;
}

void ecm_update(ECMState *state, float current, float dt) {
    float tau = state->r1 * state->c1;
    float exp_tau = expf(-dt / tau);
    state->v_rc = state->v_rc * exp_tau + fabsf(current) * state->r1 * (1.0f - exp_tau);
}