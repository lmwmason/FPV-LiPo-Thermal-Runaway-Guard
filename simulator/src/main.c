#include <stdio.h>
#include "physics.h"
#include "acro.h"
#include "trajectory.h"
#include "battery_sim.h"
#include "params.h"

#define MAX_CYCLES 500000L
#define JUMP_SAFETY_TTR 400.0f
#define RATE_PROBE_CYCLES 300

static void write_row(FILE *fout, float t, const QuadState *quad, float roll, float pitch, float yaw,
                       float throttle, const BatterySimOutput *out) {
    fprintf(fout, "%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.2f,%.6f,%.3f,%.6f,%.2f,%.2f,%d\n",
            t, quad->pos.x, quad->pos.y, quad->pos.z,
            roll * SIM_RAD2DEG, pitch * SIM_RAD2DEG, yaw * SIM_RAD2DEG,
            throttle, out->current, out->voltage, out->soc, out->soh,
            out->r0, out->temperature, out->q_sei, out->time_to_runaway,
            out->output_limit, out->status);
}

static void log_summary_row(FILE *fout, float t, const QuadState *quad, BatterySim *battery) {
    BatterySimOutput out;
    out.current = 0.0f;
    out.voltage = ecm_predict_voltage(&battery->true_ecm, 0.0f);
    out.soc = battery->ekf.x[0];
    out.soh = battery->soh;
    out.r0 = battery->ekf.x[2];
    out.temperature = battery->temperature;
    out.q_sei = battery->arrhenius.q_sei;
    out.time_to_runaway = battery->arrhenius.time_to_runaway;
    out.output_limit = battery->controller.output_limit;
    out.status = (int)battery->controller.status;

    float roll, pitch, yaw;
    quad_get_euler(quad, &roll, &pitch, &yaw);
    write_row(fout, t, quad, roll, pitch, yaw, 0.0f, &out);
}

static int run_cycle(QuadState *quad, AcroController *acro, BatterySim *battery,
                      float t_offset, FILE *fout, int do_log,
                      int use_override, float q_sei_start, float q_per_cycle) {
    quad_init(quad);
    acro_init(acro);
    float t_local = 0.0f;
    int max_status = 0;

    while (t_local < SIM_DURATION) {
        RcCommand rc;
        trajectory_get(t_local, &rc);

        float motor_cmd[4];
        acro_update(acro, &rc, quad->omega.x, quad->omega.y, quad->omega.z, SIM_DT, motor_cmd);
        quad_step(quad, motor_cmd, SIM_DT);

        BatterySimOutput out;
        battery_sim_step(battery, quad->motor_thrust, SIM_DT, &out);

        if (use_override) {
            float frac = (t_local + SIM_DT) / SIM_DURATION;
            battery->arrhenius.q_sei = q_sei_start + frac * q_per_cycle;
            arrhenius_update(&battery->arrhenius, battery->soh, battery->temperature, 0.0f);
            controller_update(&battery->controller, &battery->arrhenius, battery->soh);
            out.q_sei = battery->arrhenius.q_sei;
            out.time_to_runaway = battery->arrhenius.time_to_runaway;
            out.output_limit = battery->controller.output_limit;
            out.status = (int)battery->controller.status;
        }

        if (out.status > max_status) {
            max_status = out.status;
        }

        if (do_log) {
            float roll, pitch, yaw;
            quad_get_euler(quad, &roll, &pitch, &yaw);
            write_row(fout, t_offset + t_local, quad, roll, pitch, yaw, rc.throttle, &out);
        }

        t_local += SIM_DT;
    }
    return max_status;
}

static void run_scenario(float soh, FILE *fout) {
    QuadState quad;
    AcroController acro;
    BatterySim battery;

    quad_init(&quad);
    acro_init(&acro);
    battery_sim_init(&battery, soh);

    int target_status = (soh >= 0.8f) ? 1 : 2;
    float t_total = 0.0f;
    long cycle = 0;

    run_cycle(&quad, &acro, &battery, t_total, fout, 1, 0, 0.0f, 0.0f);
    t_total += SIM_DURATION;
    cycle++;

    if ((int)battery.controller.status >= target_status) {
        printf("  -> reached status=%d after %ld cycles (t=%.0fs, %.2f days)\n",
               battery.controller.status, cycle, t_total, t_total / 86400.0);
        return;
    }

    float q_sei_history[RATE_PROBE_CYCLES + 1];
    float ttr_history[RATE_PROBE_CYCLES + 1];
    q_sei_history[0] = battery.arrhenius.q_sei;
    ttr_history[0] = battery.arrhenius.time_to_runaway;

    for (int i = 1; i <= RATE_PROBE_CYCLES; i++) {
        run_cycle(&quad, &acro, &battery, t_total, fout, 0, 0, 0.0f, 0.0f);
        t_total += SIM_DURATION;
        cycle++;
        q_sei_history[i] = battery.arrhenius.q_sei;
        ttr_history[i] = battery.arrhenius.time_to_runaway;
        if ((int)battery.controller.status >= target_status) {
            printf("  -> reached status=%d after %ld cycles (t=%.0fs, %.2f days)\n",
                   battery.controller.status, cycle, t_total, t_total / 86400.0);
            return;
        }
    }

    float q_per_cycle = q_sei_history[RATE_PROBE_CYCLES] - q_sei_history[RATE_PROBE_CYCLES - 1];
    float ttr_drop_per_cycle = ttr_history[RATE_PROBE_CYCLES - 1] - ttr_history[RATE_PROBE_CYCLES];
    float q_sei_tracked = q_sei_history[RATE_PROBE_CYCLES];
    float ttr_now = ttr_history[RATE_PROBE_CYCLES];

    while (ttr_drop_per_cycle > 1e-9f && ttr_now > JUMP_SAFETY_TTR && cycle < MAX_CYCLES) {
        long n_jump = (long)((ttr_now - JUMP_SAFETY_TTR) / ttr_drop_per_cycle);
        if (n_jump < 1) {
            break;
        }
        for (;;) {
            ArrheniusState trial = battery.arrhenius;
            trial.q_sei = q_sei_tracked + (float)n_jump * q_per_cycle;
            arrhenius_update(&trial, soh, battery.temperature, 0.0f);
            if (trial.time_to_runaway > JUMP_SAFETY_TTR) {
                q_sei_tracked = trial.q_sei;
                battery.arrhenius = trial;
                controller_update(&battery.controller, &battery.arrhenius, soh);
                t_total += (float)n_jump * SIM_DURATION;
                cycle += n_jump;
                ttr_now = battery.arrhenius.time_to_runaway;
                log_summary_row(fout, t_total, &quad, &battery);
                break;
            }
            n_jump /= 2;
            if (n_jump < 1) {
                ttr_now = JUMP_SAFETY_TTR;
                break;
            }
        }
    }

    if ((int)battery.controller.status >= target_status) {
        printf("  -> reached status=%d after %ld cycles (t=%.0fs, %.2f days)\n",
               battery.controller.status, cycle, t_total, t_total / 86400.0);
        return;
    }

    int reached_status = 0;
    while (cycle < MAX_CYCLES) {
        int max_status = run_cycle(&quad, &acro, &battery, t_total, fout, 1, 1, q_sei_tracked, q_per_cycle);
        q_sei_tracked += q_per_cycle;
        t_total += SIM_DURATION;
        cycle++;
        if (max_status >= target_status) {
            reached_status = max_status;
            break;
        }
    }

    printf("  -> reached status=%d after %ld cycles (t=%.0fs, %.2f days)\n",
           reached_status, cycle, t_total, t_total / 86400.0);
}

int main(void) {
    char path[512];
    snprintf(path, sizeof(path), "%s/simulation.csv", OUTPUT_DIR);

    FILE *fout = fopen(path, "w");
    if (!fout) {
        printf("[Error] Cannot open output CSV.\n");
        return 1;
    }

    fprintf(fout, "time,x,y,z,roll,pitch,yaw,throttle,current,voltage,soc,soh,r0,temperature,q_sei,time_to_runaway,output_limit,status\n");

    printf("[Run] Simulating SOH=1.00 endurance scenario...\n");
    run_scenario(1.0f, fout);

    printf("[Run] Simulating SOH=0.70 endurance scenario...\n");
    run_scenario(0.7f, fout);

    fclose(fout);
    printf("[Success] Output saved to %s\n", path);
    printf("========== Done!! ==========\n");
    return 0;
}
