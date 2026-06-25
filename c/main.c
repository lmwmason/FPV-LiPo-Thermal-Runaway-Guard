#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ecm.h"
#include "ekf.h"
#include "arrhenius.h"
#include "controller.h"

#define MAX_ROWS 100000
#define SOH_R0_SCALE 3.0f

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

    FILE *fout = fopen("data/output.csv", "w");
    if (!fout) {
        printf("[Error] Cannot open output CSV.\n");
        free(rows);
        return 1;
    }
    fprintf(fout, "cycle,time,voltage,soc,soh,r0,time_to_runaway,output_limit,status\n");

    ECMState ecm;
    EKFState ekf;
    ArrheniusState arrhenius;
    ControllerState controller;

    ecm_init(&ecm, 1.0f);
    ekf_init(&ekf, &ecm);
    arrhenius_init(&arrhenius);
    controller_init(&controller);

    int prev_cycle = -1;
    float prev_time = 0.0f;
    float r0_cycle_sum = 0.0f;
    int r0_cycle_count = 0;
    float r0_first_cycle = 0.0f;
    float soh = 1.0f;
    int first_cycle_done = 0;

    for (int i = 0; i < row_count; i++) {
        Row *r = &rows[i];

        if (r->cycle != prev_cycle) {
            if (prev_cycle >= 0 && r0_cycle_count > 0) {
                float r0_avg = r0_cycle_sum / r0_cycle_count;
                if (!first_cycle_done) {
                    r0_first_cycle = r0_avg;
                    first_cycle_done = 1;
                }
                soh = fminf(1.0f, r0_first_cycle / fmaxf(r0_avg, 1e-4f));
            }
            ecm_init(&ecm, 1.0f);
            ekf_init(&ekf, &ecm);
            prev_cycle = r->cycle;
            prev_time = r->time;
            r0_cycle_sum = 0.0f;
            r0_cycle_count = 0;
        }

        float dt = (r->cycle != rows[i == 0 ? 0 : i-1].cycle || i == 0) ? 0.0f : r->time - prev_time;
        if (dt < 0.0f) dt = 0.0f;
        prev_time = r->time;

        if (dt > 0.0f)
            ekf_update(&ekf, r->voltage, r->current, dt);

        r0_cycle_sum += ekf.x[2];
        r0_cycle_count++;

        arrhenius_update(&arrhenius, soh, r->temperature, dt);
        controller_update(&controller, &arrhenius, soh);

        fprintf(fout, "%d,%.2f,%.4f,%.4f,%.4f,%.6f,%.2f,%.2f,%d\n",
            r->cycle, r->time, r->voltage,
            ekf.x[0], soh, ekf.x[2],
            arrhenius.time_to_runaway,
            controller.output_limit,
            controller.status);
    }

    fclose(fout);
    free(rows);
    printf("[Success] Output saved to data/output.csv\n");
    printf("========== Done!! ==========\n");
    return 0;
}