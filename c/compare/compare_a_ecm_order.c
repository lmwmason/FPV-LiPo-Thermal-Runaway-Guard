#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ecm.h"
#include "ecm2.h"

#define MAX_ROWS 100000
#define Q_NOMINAL (2.0f * 3600.0f)

typedef struct {
    int cycle;
    float time;
    float voltage;
    float current;
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
        float temperature;
        if (sscanf(line, "%d,%f,%f,%f,%f", &r.cycle, &r.time, &r.voltage, &r.current, &temperature) == 5) {
            rows[row_count++] = r;
        }
    }
    fclose(fin);
    printf("[Load] Loaded %d rows.\n", row_count);

    FILE *fout = fopen("data/compare_a.csv", "w");
    if (!fout) {
        printf("[Error] Cannot open output CSV.\n");
        free(rows);
        return 1;
    }
    fprintf(fout, "cycle,time,voltage_measured,voltage_1rc,voltage_2rc\n");

    ECMState ecm;
    ECM2State ecm2;
    ecm_init(&ecm, 1.0f);
    ecm2_init(&ecm2, 1.0f);

    int prev_cycle = -1;
    float prev_time = 0.0f;

    for (int i = 0; i < row_count; i++) {
        Row *r = &rows[i];

        if (r->cycle != prev_cycle) {
            ecm_init(&ecm, 1.0f);
            ecm2_init(&ecm2, 1.0f);
            prev_cycle = r->cycle;
            prev_time = r->time;
        }

        float dt = (i == 0 || rows[i - 1].cycle != r->cycle) ? 0.0f : r->time - prev_time;
        if (dt < 0.0f) dt = 0.0f;
        prev_time = r->time;

        float i_abs = fabsf(r->current);
        float soc_next = ecm.soc - (i_abs * dt) / Q_NOMINAL;
        soc_next = fmaxf(0.0f, fminf(1.0f, soc_next));
        ecm.soc = soc_next;
        ecm2.soc = soc_next;

        ecm_update(&ecm, r->current, dt);
        ecm2_update(&ecm2, r->current, dt);

        float v_1rc = ecm_predict_voltage(&ecm, r->current);
        float v_2rc = ecm2_predict_voltage(&ecm2, r->current);

        fprintf(fout, "%d,%.2f,%.4f,%.4f,%.4f\n", r->cycle, r->time, r->voltage, v_1rc, v_2rc);
    }

    fclose(fout);
    free(rows);
    printf("[Success] Output saved to data/compare_a.csv\n");
    printf("========== Done!! ==========\n");
    return 0;
}
