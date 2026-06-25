#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ecm.h"
#include "ekf.h"
#include "arrhenius.h"

#define MAX_ROWS 100000
#define MAX_CYCLES 200
#define TEMP_THRESHOLD 40.0f
#define ARRHENIUS_ALERT_RATIO 0.5f

typedef struct {
    int cycle;
    float time;
    float voltage;
    float current;
    float temperature;
} Row;

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

    float simple_alert_time[MAX_CYCLES];
    float arrhenius_alert_time[MAX_CYCLES];
    for (int i = 0; i <= max_cycle; i++) {
        simple_alert_time[i] = -1.0f;
        arrhenius_alert_time[i] = -1.0f;
    }

    ArrheniusState arrhenius;
    arrhenius_init(&arrhenius);

    prev_cycle = -1;
    prev_time = 0.0f;
    float baseline_ttr = 1e9f;

    for (int i = 0; i < row_count; i++) {
        Row *r = &rows[i];
        if (r->cycle < 0 || r->cycle >= MAX_CYCLES) continue;

        if (r->cycle != prev_cycle) {
            prev_cycle = r->cycle;
            prev_time = r->time;
        }

        float dt = (i == 0 || rows[i - 1].cycle != r->cycle) ? 0.0f : r->time - prev_time;
        if (dt < 0.0f) dt = 0.0f;
        prev_time = r->time;

        arrhenius_update(&arrhenius, soh_r0[r->cycle], r->temperature, dt);

        if (r->cycle == 0 && dt > 0.0f && arrhenius.time_to_runaway < baseline_ttr) {
            baseline_ttr = arrhenius.time_to_runaway;
        }

        if (r->temperature > TEMP_THRESHOLD && simple_alert_time[r->cycle] < 0.0f) {
            simple_alert_time[r->cycle] = r->time;
        }

        if (r->cycle > 0 &&
            arrhenius.time_to_runaway <= baseline_ttr * ARRHENIUS_ALERT_RATIO &&
            arrhenius_alert_time[r->cycle] < 0.0f) {
            arrhenius_alert_time[r->cycle] = r->time;
        }
    }
    free(rows);

    FILE *fout = fopen("data/compare_d.csv", "w");
    if (!fout) {
        printf("[Error] Cannot open output CSV.\n");
        return 1;
    }
    fprintf(fout, "cycle,simple_alert_time,arrhenius_alert_time\n");
    for (int c = 0; c <= max_cycle; c++) {
        fprintf(fout, "%d,%.2f,%.2f\n", c, simple_alert_time[c], arrhenius_alert_time[c]);
    }
    fclose(fout);

    printf("[Info] baseline_ttr=%.2f\n", baseline_ttr);
    printf("[Success] Output saved to data/compare_d.csv\n");
    printf("========== Done!! ==========\n");
    return 0;
}
