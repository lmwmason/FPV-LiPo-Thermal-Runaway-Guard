#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ecm.h"
#include "ekf.h"

#define MAX_ROWS 100000
#define MAX_CYCLES 200

typedef struct {
    int cycle;
    float time;
    float voltage;
    float current;
} Row;

int main(void) {
    FILE *fcap = fopen("data/B0005_capacity.csv", "r");
    if (!fcap) {
        printf("[Error] Cannot open capacity CSV.\n");
        return 1;
    }
    float capacity[MAX_CYCLES];
    for (int i = 0; i < MAX_CYCLES; i++) capacity[i] = 0.0f;
    char line[256];
    fgets(line, sizeof(line), fcap);
    while (fgets(line, sizeof(line), fcap)) {
        int c;
        float cap;
        if (sscanf(line, "%d,%f", &c, &cap) == 2 && c >= 0 && c < MAX_CYCLES) {
            capacity[c] = cap;
        }
    }
    fclose(fcap);

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

    fgets(line, sizeof(line), fin);
    int row_count = 0;
    while (fgets(line, sizeof(line), fin) && row_count < MAX_ROWS) {
        Row r;
        float temperature;
        if (sscanf(line, "%d,%f,%f,%f,%f", &r.cycle, &r.time, &r.voltage, &r.current, &temperature) == 5) {
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
    free(rows);

    FILE *fout = fopen("data/compare_c.csv", "w");
    if (!fout) {
        printf("[Error] Cannot open output CSV.\n");
        return 1;
    }
    fprintf(fout, "cycle,soh_r0,soh_capacity\n");

    float r0_baseline = (r0_count[0] > 0) ? r0_sum[0] / r0_count[0] : 1.0f;
    float capacity_baseline = (capacity[0] > 0.0f) ? capacity[0] : 1.0f;

    for (int c = 0; c <= max_cycle; c++) {
        if (r0_count[c] == 0) continue;
        float r0_avg = r0_sum[c] / r0_count[c];
        float soh_r0 = fminf(1.0f, r0_baseline / fmaxf(r0_avg, 1e-4f));
        float soh_capacity = capacity[c] / capacity_baseline;
        fprintf(fout, "%d,%.4f,%.4f\n", c, soh_r0, soh_capacity);
    }

    fclose(fout);
    printf("[Success] Output saved to data/compare_c.csv\n");
    printf("========== Done!! ==========\n");
    return 0;
}
