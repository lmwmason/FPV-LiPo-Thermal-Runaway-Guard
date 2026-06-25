#include "ekf.h"
#include "ecm.h"
#include "ecm_params.h"
#include <math.h>
#include <string.h>

#define Q_NOMINAL (2.0f * 3600.0f)

static void mat_mul_5x5(float out[5][5], float a[5][5], float b[5][5]) {
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++) {
            out[i][j] = 0.0f;
            for (int k = 0; k < 5; k++)
                out[i][j] += a[i][k] * b[k][j];
        }
}

static void mat_transpose_5x5(float out[5][5], float a[5][5]) {
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++)
            out[i][j] = a[j][i];
}

void ekf_init(EKFState *ekf, const ECMState *ecm) {
    ekf->x[0] = ecm->soc;
    ekf->x[1] = ecm->v_rc;
    ekf->x[2] = ecm->r0;
    ekf->x[3] = ecm->r1;
    ekf->x[4] = ecm->c1;

    memset(ekf->P, 0, sizeof(ekf->P));
    ekf->P[0][0] = 0.01f;
    ekf->P[1][1] = 0.01f;
    ekf->P[2][2] = 0.01f;
    ekf->P[3][3] = 0.01f;
    ekf->P[4][4] = 100.0f;

    memset(ekf->Q, 0, sizeof(ekf->Q));
    ekf->Q[0][0] = 1e-6f;
    ekf->Q[1][1] = 1e-5f;
    ekf->Q[2][2] = 7e-7f;
    ekf->Q[3][3] = 1e-8f;
    ekf->Q[4][4] = 1e-2f;

    ekf->R = 1e-6f;
}

void ekf_update(EKFState *ekf, float voltage, float current, float dt) {
    float soc = ekf->x[0];
    float v_rc = ekf->x[1];
    float r0   = ekf->x[2];
    float r1   = ekf->x[3];
    float c1   = ekf->x[4];
    float i_abs = fabsf(current);

    float tau = r1 * c1;
    float exp_tau = (tau > 0.0f) ? expf(-dt / tau) : 0.0f;

    float x_pred[5];
    x_pred[0] = soc - (i_abs * dt) / Q_NOMINAL;
    if (x_pred[0] < 0.0f) x_pred[0] = 0.0f;
    if (x_pred[0] > 1.0f) x_pred[0] = 1.0f;
    x_pred[1] = v_rc * exp_tau + i_abs * r1 * (1.0f - exp_tau);
    x_pred[2] = r0;
    x_pred[3] = r1;
    x_pred[4] = c1;

    float F[5][5];
    memset(F, 0, sizeof(F));
    F[0][0] = 1.0f;
    F[1][1] = exp_tau;
    F[1][3] = i_abs * (1.0f - exp_tau);
    F[1][4] = (tau > 0.0f) ? i_abs * r1 * exp_tau * dt / (tau * c1) : 0.0f;
    F[2][2] = 1.0f;
    F[3][3] = 1.0f;
    F[4][4] = 1.0f;

    float docv_dsoc = (ecm_ocv_lookup(soc + 1e-4f) - ecm_ocv_lookup(soc - 1e-4f)) / 2e-4f;
    float H[5] = {docv_dsoc, -1.0f, -i_abs, 0.0f, 0.0f};

    float FT[5][5];
    mat_transpose_5x5(FT, F);
    float FP[5][5];
    mat_mul_5x5(FP, F, ekf->P);
    float P_pred[5][5];
    mat_mul_5x5(P_pred, FP, FT);
    for (int i = 0; i < 5; i++)
        P_pred[i][i] += ekf->Q[i][i];

    float PH[5] = {0};
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++)
            PH[i] += P_pred[i][j] * H[j];

    float S = 0.0f;
    for (int i = 0; i < 5; i++)
        S += H[i] * PH[i];
    S += ekf->R;

    float K[5];
    for (int i = 0; i < 5; i++)
        K[i] = PH[i] / S;

    float ocv = ecm_ocv_lookup(x_pred[0]);
    float v_pred = ocv - i_abs * x_pred[2] - x_pred[1];
    float y = voltage - v_pred;

    for (int i = 0; i < 5; i++)
        ekf->x[i] = x_pred[i] + K[i] * y;

    ekf->x[0] = fmaxf(0.0f, fminf(1.0f, ekf->x[0]));
    ekf->x[2] = fmaxf(1e-4f, ekf->x[2]);
    ekf->x[3] = fmaxf(1e-4f, ekf->x[3]);
    ekf->x[4] = fmaxf(1.0f,  ekf->x[4]);

    float KH[5][5];
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++)
            KH[i][j] = K[i] * H[j];

    float IKH[5][5];
    memset(IKH, 0, sizeof(IKH));
    for (int i = 0; i < 5; i++)
        IKH[i][i] = 1.0f;
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++)
            IKH[i][j] -= KH[i][j];

    mat_mul_5x5(ekf->P, IKH, P_pred);
}