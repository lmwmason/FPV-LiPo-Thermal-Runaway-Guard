#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ecm.h"
#include "ekf.h"

#define MAX_ROWS 100000
#define MAX_CYCLES 200
#define Q_NOMINAL (2.0f * 3600.0f)

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
    for (int i = 0; i < MAX_CYCLES; i++) capacity[i] = 1.85f;
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

    FILE *fout = fopen("data/compare_b.csv", "w");
    if (!fout) {
        printf("[Error] Cannot open output CSV.\n");
        free(rows);
        return 1;
    }
    fprintf(fout, "cycle,time,soc_true,soc_coulomb,soc_ekf\n");

    ECMState ecm;
    EKFState ekf;
    ecm_init(&ecm, 1.0f);
    ekf_init(&ekf, &ecm);

    int prev_cycle = -1;
    float prev_time = 0.0f;
    float cap_as = capacity[0] * 3600.0f;
    float soc_true = 1.0f;
    float soc_coulomb = 1.0f;

    for (int i = 0; i < row_count; i++) {
        Row *r = &rows[i];

        if (r->cycle != prev_cycle) {
            ecm_init(&ecm, 1.0f);
            ekf_init(&ekf, &ecm);
            prev_cycle = r->cycle;
            prev_time = r->time;
            int c = (r->cycle >= 0 && r->cycle < MAX_CYCLES) ? r->cycle : 0;
            cap_as = capacity[c] * 3600.0f;
            soc_true = 1.0f;
            soc_coulomb = 1.0f;
        }

        float dt = (i == 0 || rows[i - 1].cycle != r->cycle) ? 0.0f : r->time - prev_time;
        if (dt < 0.0f) dt = 0.0f;
        prev_time = r->time;

        float i_abs = fabsf(r->current);
        soc_true -= (i_abs * dt) / cap_as;
        soc_true = fmaxf(0.0f, fminf(1.0f, soc_true));

        soc_coulomb -= (i_abs * dt) / Q_NOMINAL;
        soc_coulomb = fmaxf(0.0f, fminf(1.0f, soc_coulomb));

        if (dt > 0.0f)
            ekf_update(&ekf, r->voltage, r->current, dt);

        fprintf(fout, "%d,%.2f,%.4f,%.4f,%.4f\n", r->cycle, r->time, soc_true, soc_coulomb, ekf.x[0]);
    }

    fclose(fout);
    free(rows);
    printf("[Success] Output saved to data/compare_b.csv\n");
    printf("========== Done!! ==========\n");
    return 0;
}
