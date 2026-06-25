#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ecm.h"
#include "ekf.h"
#include "arrhenius.h"

#define MAX_ROWS 100000
#define MAX_CYCLES 200
#define ARRHENIUS_WARNING_RATIO 0.5f
#define ARRHENIUS_CRITICAL_RATIO 0.2f

typedef enum {
    STATUS_NORMAL,
    STATUS_WARNING,
    STATUS_CRITICAL,
} Status;

typedef struct {
    int cycle;
    float time;
    float voltage;
    float current;
    float temperature;
} Row;

static Status decide_status(float time_to_runaway, float baseline_ttr) {
    if (time_to_runaway <= baseline_ttr * ARRHENIUS_CRITICAL_RATIO)
        return STATUS_CRITICAL;
    if (time_to_runaway <= baseline_ttr * ARRHENIUS_WARNING_RATIO)
        return STATUS_WARNING;
    return STATUS_NORMAL;
}

static float status_output_limit(Status status) {
    if (status == STATUS_CRITICAL) return 0.0f;
    if (status == STATUS_WARNING) return 0.7f;
    return 1.0f;
}

int main(void) {
    FILE *fin = fopen("data/B0005_discharge.csv", "r");
    if (!fin) {
        printf("[Error] Cannot open input CSV.\n");
        return 1;
    }

    Row *rows = malloc(sizeof(Row) * MAX_ROWS);
    if (!rows) {
        printf("[Error] Memory allocation failed.\n");
        fclose(fin);
        return 1;
    }

    char line[256];
    fgets(line, sizeof(line), fin);
    int row_count = 0;
    while (fgets(line, sizeof(line), fin) && row_count < MAX_ROWS) {
        Row r;
        if (sscanf(line, "%d,%f,%f,%f,%f", &r.cycle, &r.time, &r.voltage, &r.current, &r.temperature) == 5) {
            rows[row_count++] = r;
        }
    }
    fclose(fin);
    printf("[Load] Loaded %d rows.\n", row_count);

    float r0_sum[MAX_CYCLES];
    int r0_count[MAX_CYCLES];
    for (int i = 0; i < MAX_CYCLES; i++) {
        r0_sum[i] = 0.0f;
        r0_count[i] = 0;
    }

    ECMState ecm;
    EKFState ekf;
    ecm_init(&ecm, 1.0f);
    ekf_init(&ekf, &ecm);

    int prev_cycle = -1;
    float prev_time = 0.0f;
    int max_cycle = 0;

    for (int i = 0; i < row_count; i++) {
        Row *r = &rows[i];
        if (r->cycle < 0 || r->cycle >= MAX_CYCLES) continue;
        if (r->cycle > max_cycle) max_cycle = r->cycle;

        if (r->cycle != prev_cycle) {
            ecm_init(&ecm, 1.0f);
            ekf_init(&ekf, &ecm);
            prev_cycle = r->cycle;
            prev_time = r->time;
        }

        float dt = (i == 0 || rows[i - 1].cycle != r->cycle) ? 0.0f : r->time - prev_time;
        if (dt < 0.0f) dt = 0.0f;
        prev_time = r->time;

        if (dt > 0.0f)
            ekf_update(&ekf, r->voltage, r->current, dt);

        r0_sum[r->cycle] += ekf.x[2];
        r0_count[r->cycle]++;
    }

    float soh_r0[MAX_CYCLES];
    float r0_baseline = (r0_count[0] > 0) ? r0_sum[0] / r0_count[0] : 1.0f;
    for (int c = 0; c <= max_cycle; c++) {
        if (r0_count[c] == 0) {
            soh_r0[c] = 1.0f;
            continue;
        }
        float r0_avg = r0_sum[c] / r0_count[c];
        soh_r0[c] = fminf(1.0f, r0_baseline / fmaxf(r0_avg, 1e-4f));
    }

    float output_limit_sum[MAX_CYCLES];
    int output_limit_count[MAX_CYCLES];
    float risk_time_fixed[MAX_CYCLES];
    float risk_time_adaptive[MAX_CYCLES];
    for (int i = 0; i <= max_cycle; i++) {
        output_limit_sum[i] = 0.0f;
        output_limit_count[i] = 0;
        risk_time_fixed[i] = 0.0f;
        risk_time_adaptive[i] = 0.0f;
    }

    ArrheniusState arr_fixed, arr_adaptive;
    arrhenius_init(&arr_fixed);
    arrhenius_init(&arr_adaptive);

    prev_cycle = -1;
    prev_time = 0.0f;
    float output_limit_prev = 1.0f;
    float ambient_temp = rows[0].temperature;
    float baseline_ttr = 1e9f;

    for (int i = 0; i < row_count; i++) {
        Row *r = &rows[i];
        if (r->cycle < 0 || r->cycle >= MAX_CYCLES) continue;

        if (r->cycle != prev_cycle) {
            prev_cycle = r->cycle;
            prev_time = r->time;
            ambient_temp = r->temperature;
        }

        float dt = (i == 0 || rows[i - 1].cycle != r->cycle) ? 0.0f : r->time - prev_time;
        if (dt < 0.0f) dt = 0.0f;
        prev_time = r->time;

        arrhenius_update(&arr_fixed, soh_r0[r->cycle], r->temperature, dt);
        if (r->cycle == 0 && dt > 0.0f && arr_fixed.time_to_runaway < baseline_ttr) {
            baseline_ttr = arr_fixed.time_to_runaway;
        }
        Status status_fixed = decide_status(arr_fixed.time_to_runaway, baseline_ttr);
        if (status_fixed != STATUS_NORMAL) risk_time_fixed[r->cycle] += dt;

        float temp_eff = ambient_temp + (r->temperature - ambient_temp) * output_limit_prev * output_limit_prev;
        arrhenius_update(&arr_adaptive, soh_r0[r->cycle], temp_eff, dt);
        Status status_adaptive = decide_status(arr_adaptive.time_to_runaway, baseline_ttr);
        if (status_adaptive != STATUS_NORMAL) risk_time_adaptive[r->cycle] += dt;

        float output_limit = status_output_limit(status_adaptive);
        output_limit_sum[r->cycle] += output_limit;
        output_limit_count[r->cycle]++;
        output_limit_prev = output_limit;
    }
    free(rows);

    FILE *fout = fopen("data/compare_e.csv", "w");
    if (!fout) {
        printf("[Error] Cannot open output CSV.\n");
        return 1;
    }
    fprintf(fout, "cycle,output_limit_avg,risk_time_fixed,risk_time_adaptive,cum_risk_time_fixed,cum_risk_time_adaptive\n");

    float cum_fixed = 0.0f;
    float cum_adaptive = 0.0f;
    for (int c = 0; c <= max_cycle; c++) {
        if (output_limit_count[c] == 0) continue;
        float output_limit_avg = output_limit_sum[c] / output_limit_count[c];
        cum_fixed += risk_time_fixed[c];
        cum_adaptive += risk_time_adaptive[c];
        fprintf(fout, "%d,%.4f,%.2f,%.2f,%.2f,%.2f\n",
            c, output_limit_avg, risk_time_fixed[c], risk_time_adaptive[c], cum_fixed, cum_adaptive);
    }

    fclose(fout);
    printf("[Info] baseline_ttr=%.2f\n", baseline_ttr);
    printf("[Success] Output saved to data/compare_e.csv\n");
    printf("========== Done!! ==========\n");
    return 0;
}
