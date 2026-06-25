import csv
import os
import numpy as np

input_dir = './output/discharge'
output_path = './output/ecm2_params.h'
MAX_CYCLE = 10
Q_NOMINAL = 2.0 * 3600.0
V_CUTOFF = 2.5

ECM_R0 = 0.102240
ECM_R1 = 0.005599
ECM_C1 = 2624.5723
ECM_R2_INIT = 0.05
ECM_C2_INIT = 20000.0

OCV_SOC_TABLE = [
    (0.0, 2.8100), (0.1, 3.2637), (0.2, 3.6115), (0.3, 3.6801),
    (0.4, 3.7134), (0.5, 3.7523), (0.6, 3.8174), (0.7, 3.8789),
    (0.8, 3.9558), (0.9, 4.0470), (1.0, 4.1915),
]
SOC_ARR = np.array([p[0] for p in OCV_SOC_TABLE])
OCV_ARR = np.array([p[1] for p in OCV_SOC_TABLE])

def ocv_lookup(soc):
    soc = max(0.0, min(1.0, soc))
    return float(np.interp(soc, SOC_ARR, OCV_ARR))

cycles = {}
try:
    print("[Load] Loading discharge CSVs...")
    battery_files = sorted([f for f in os.listdir(input_dir) if f.endswith('.csv')])
    for battery_file in battery_files:
        with open(os.path.join(input_dir, battery_file), 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                if int(row['cycle']) > MAX_CYCLE:
                    continue
                cycle_id = f"{battery_file}_{row['cycle']}"
                if cycle_id not in cycles:
                    cycles[cycle_id] = []
                cycles[cycle_id].append((float(row['time']), float(row['voltage']), float(row['current'])))
    print(f"[Success] Loaded {len(cycles)} cycles.")
except Exception as e:
    print(f'[Error] {e}')
    exit(1)

try:
    print("[Load] Running EKF to identify only the second RC branch (R0,R1,C1 kept fixed from 1RC fit)...")
    # state x = [soc, v1, v2, r2, c2]
    Q_noise = np.diag([1e-6, 1e-5, 1e-5, 1e-8, 1e-2])
    R_noise = np.array([[1e-4]])
    all_r2, all_c2 = [], []

    for cycle_id in sorted(cycles.keys()):
        samples = cycles[cycle_id]
        if len(samples) < 10:
            continue

        x = np.array([1.0, 0.0, 0.0, ECM_R2_INIT, ECM_C2_INIT])
        P = np.diag([0.01, 0.01, 0.01, 0.01, 100.0])

        for idx, (time, voltage, current) in enumerate(samples):
            if voltage < V_CUTOFF:
                break
            if idx == 0:
                dt = 0.0
            else:
                dt = time - samples[idx - 1][0]
                if dt <= 0:
                    continue

            soc, v1, v2, r2, c2 = x
            i_abs = abs(current)

            tau1 = ECM_R1 * ECM_C1
            exp1 = np.exp(-dt / tau1) if tau1 > 0 else 0.0
            tau2 = r2 * c2
            exp2 = np.exp(-dt / tau2) if tau2 > 0 else 0.0

            soc_new = soc - (i_abs * dt) / Q_NOMINAL
            v1_new = v1 * exp1 + i_abs * ECM_R1 * (1.0 - exp1)
            v2_new = v2 * exp2 + i_abs * r2 * (1.0 - exp2)
            x_pred = np.array([np.clip(soc_new, 0.0, 1.0), v1_new, v2_new, r2, c2])

            docv_dsoc = (ocv_lookup(soc + 1e-4) - ocv_lookup(soc - 1e-4)) / 2e-4
            H = np.array([[docv_dsoc, -1.0, -1.0, 0.0, 0.0]])

            F = np.eye(5)
            F[1, 1] = exp1
            F[2, 2] = exp2
            F[2, 3] = i_abs * (1.0 - exp2)
            F[2, 4] = i_abs * r2 * exp2 * dt / (tau2 * c2) if tau2 > 0 else 0.0

            P_pred = F @ P @ F.T + Q_noise
            S = H @ P_pred @ H.T + R_noise
            K = P_pred @ H.T @ np.linalg.inv(S)

            ocv = ocv_lookup(x_pred[0])
            v_pred = ocv - i_abs * ECM_R0 - x_pred[1] - x_pred[2]
            y = voltage - v_pred

            x = x_pred + (K @ np.array([y])).flatten()
            x[0] = np.clip(x[0], 0.0, 1.0)
            x[3] = max(x[3], 1e-4)
            x[4] = max(x[4], 1.0)
            P = (np.eye(5) - K @ H) @ P_pred

        all_r2.append(x[3])
        all_c2.append(x[4])

    R0, R1, C1 = ECM_R0, ECM_R1, ECM_C1
    R2 = float(np.median(all_r2))
    C2 = float(np.median(all_c2))
    TAU1 = R1 * C1
    TAU2 = R2 * C2
    print(f"[Success] Fitted {len(all_r2)} cycles. R2={R2:.6f} C2={C2:.4f} (R0={R0:.6f} R1={R1:.6f} C1={C1:.4f} kept fixed)")
except Exception as e:
    print(f'[Error] {e}')
    exit(1)

try:
    print("[Load] Writing ecm2_params.h...")
    with open(output_path, 'w') as f:
        f.write("#pragma once\n\n")
        f.write(f"#define ECM2_R0_INIT   {R0:.6f}f\n")
        f.write(f"#define ECM2_R1_INIT   {R1:.6f}f\n")
        f.write(f"#define ECM2_C1_INIT   {C1:.4f}f\n")
        f.write(f"#define ECM2_TAU1_INIT {TAU1:.4f}f\n")
        f.write(f"#define ECM2_R2_INIT   {R2:.6f}f\n")
        f.write(f"#define ECM2_C2_INIT   {C2:.4f}f\n")
        f.write(f"#define ECM2_TAU2_INIT {TAU2:.4f}f\n")
    print("[Success] ecm2_params.h saved!")
except Exception as e:
    print(f'[Error] {e}')
    exit(1)

print("========== Done!! ==========")
