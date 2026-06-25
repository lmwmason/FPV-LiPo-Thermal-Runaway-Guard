#pragma once

#include "ecm.h"

typedef struct {
    float x[5];
    float P[5][5];
    float Q[5][5];
    float R;
} EKFState;

void ekf_init(EKFState *ekf, const ECMState *ecm);
void ekf_update(EKFState *ekf, float voltage, float current, float dt);