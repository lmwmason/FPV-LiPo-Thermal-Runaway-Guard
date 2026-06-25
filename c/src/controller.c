#include "controller.h"

#define TIME_WARNING   300.0f
#define TIME_CRITICAL  100.0f
#define SOH_WARNING    0.8f
#define SOH_CRITICAL   0.6f

void controller_init(ControllerState *state) {
    state->status = CONTROLLER_NORMAL;
    state->output_limit = 1.0f;
}

void controller_update(ControllerState *state, const ArrheniusState *arrhenius, float soh) {
    float time_to_runaway = arrhenius_time_to_runaway(arrhenius);

    if (time_to_runaway <= TIME_CRITICAL || soh <= SOH_CRITICAL) {
        state->status = CONTROLLER_CRITICAL;
        state->output_limit = 0.0f;
    } else if (time_to_runaway <= TIME_WARNING || soh <= SOH_WARNING) {
        state->status = CONTROLLER_WARNING;
        state->output_limit = 0.7f;
    } else {
        state->status = CONTROLLER_NORMAL;
        state->output_limit = 1.0f;
    }
}

float controller_get_output_limit(const ControllerState *state) {
    return state->output_limit;
}