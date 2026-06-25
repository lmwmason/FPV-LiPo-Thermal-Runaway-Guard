#include "ecm2.h"
#include "ecm.h"
#include "ecm2_params.h"
#include <math.h>

void ecm2_init(ECM2State *state, float soc_init) {
    state->soc = soc_init;
    state->v1 = 0.0f;
    state->v2 = 0.0f;
    state->r0 = ECM2_R0_INIT;
    state->r1 = ECM2_R1_INIT;
    state->c1 = ECM2_C1_INIT;
    state->r2 = ECM2_R2_INIT;
    state->c2 = ECM2_C2_INIT;
}

float ecm2_predict_voltage(const ECM2State *state, float current) {
    float ocv = ecm_ocv_lookup(state->soc);
    return ocv - fabsf(current) * state->r0 - state->v1 - state->v2;
}

void ecm2_update(ECM2State *state, float current, float dt) {
    float i_abs = fabsf(current);

    float tau1 = state->r1 * state->c1;
    float exp_tau1 = expf(-dt / tau1);
    state->v1 = state->v1 * exp_tau1 + i_abs * state->r1 * (1.0f - exp_tau1);

    float tau2 = state->r2 * state->c2;
    float exp_tau2 = expf(-dt / tau2);
    state->v2 = state->v2 * exp_tau2 + i_abs * state->r2 * (1.0f - exp_tau2);
}
