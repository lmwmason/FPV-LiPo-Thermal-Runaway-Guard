#pragma once

#include "arrhenius.h"

typedef enum {
    CONTROLLER_NORMAL,
    CONTROLLER_WARNING,
    CONTROLLER_CRITICAL,
} ControllerStatus;

typedef struct {
    ControllerStatus status;
    float output_limit;
} ControllerState;

void controller_init(ControllerState *state);
void controller_update(ControllerState *state, const ArrheniusState *arrhenius, float soh);
float controller_get_output_limit(const ControllerState *state);