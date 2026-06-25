import csv
import os
import numpy as np

input_dir = './output/discharge'
output_path = './output/ecm_params.h'
MAX_CYCLE = 10
I_THRESHOLD = 0.5
Q_NOMINAL = 2.0 * 3600.0
V_CUTOFF = 2.5

OCV_SOC_TABLE = [
    (0.0, 2.8100), (0.1, 3.2637), (0.2, 3.6115), (0.3, 3.6801),
    (0.4, 3.7134), (0.5, 3.7523), (0.6, 3.8174), (0.7, 3.8789),
    (0.8, 3.9558), (0.9, 4.0470), (1.0, 4.1915),
]

def ocv_lookup(soc):
    soc = max(0.0, min(1.0, soc))
    soc_arr = np.array([p[0] for p in OCV_SOC_TABLE])
    ocv_arr = np.array([p[1] for p in OCV_SOC_TABLE])
    return float(np.interp(soc, soc_arr, ocv_arr))

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
    print("[Load] Extracting initial ECM parameters...")
    r0_list = []
    r1_list = []
    tau_list = []
    for cycle_id in sorted(cycles.keys()):
        samples = cycles[cycle_id]
        if len(samples) < 5:
            continue
        transition_idx = None
        for idx in range(1, len(samples)):
            if abs(samples[idx][2]) > I_THRESHOLD and abs(samples[idx-1][2]) < I_THRESHOLD:
                transition_idx = idx
                break
        if transition_idx is None:
            continue
        v_before = samples[transition_idx - 1][1]
        v_after = samples[transition_idx][1]
        i_after = abs(samples[transition_idx][2])
        r0 = (v_before - v_after) / i_after
        if r0 <= 0 or r0 > 1.0:
            continue
        r0_list.append(r0)
        v_steady = samples[-1][1]
        v_drop_rc = v_after - v_steady
        if v_drop_rc <= 0:
            continue
        r1 = v_drop_rc / i_after
        if r1 <= 0 or r1 > 1.0:
            continue
        r1_list.append(r1)
        dt = samples[-1][0] - samples[transition_idx][0]
        tau_list.append(dt / 3.0)
    R0 = float(np.median(r0_list)) if r0_list else 0.1
    R1 = float(np.median(r1_list)) if r1_list else 0.05
    TAU = float(np.median(tau_list)) if tau_list else 60.0
    C1 = TAU / R1
    print("[Success] Initial ECM parameters extracted.")
except Exception as e:
    print(f'[Error] {e}')
    exit(1)

try:
    print("[Load] Running EKF to refine ECM parameters...")
    Q_noise = np.diag([1e-6, 1e-5, 1e-8, 1e-8, 1e-2])
    R_noise = np.array([[1e-4]])
    all_r0 = []
    all_r1 = []
    all_c1 = []

    for cycle_id in sorted(cycles.keys()):
        samples = cycles[cycle_id]
        if len(samples) < 10:
            continue

        x = np.array([1.0, 0.0, R0, R1, C1])
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

            soc, v_rc, r0, r1, c1 = x
            tau = r1 * c1
            exp_tau = np.exp(-dt / tau) if tau > 0 else 0.0

            soc_new = soc - (abs(current) * dt) / Q_NOMINAL
            v_rc_new = v_rc * exp_tau + abs(current) * r1 * (1.0 - exp_tau)
            x_pred = np.array([np.clip(soc_new, 0.0, 1.0), v_rc_new, r0, r1, c1])

            docv_dsoc = (ocv_lookup(soc + 1e-4) - ocv_lookup(soc - 1e-4)) / 2e-4
            H = np.array([[docv_dsoc, -1.0, -abs(current), 0.0, 0.0]])

            F = np.eye(5)
            F[1, 1] = exp_tau
            F[1, 3] = abs(current) * (1.0 - exp_tau)
            F[1, 4] = abs(current) * r1 * exp_tau * dt / (tau * c1) if tau > 0 else 0.0

            P_pred = F @ P @ F.T + Q_noise
            S = H @ P_pred @ H.T + R_noise
            K = P_pred @ H.T @ np.linalg.inv(S)

            ocv = ocv_lookup(x_pred[0])
            v_pred = ocv - abs(current) * x_pred[2] - x_pred[1]
            y = voltage - v_pred

            x = x_pred + (K @ np.array([y])).flatten()
            x[0] = np.clip(x[0], 0.0, 1.0)
            x[2] = max(x[2], 1e-4)
            x[3] = max(x[3], 1e-4)
            x[4] = max(x[4], 1.0)
            P = (np.eye(5) - K @ H) @ P_pred

        all_r0.append(x[2])
        all_r1.append(x[3])
        all_c1.append(x[4])

    R0 = float(np.median(all_r0))
    R1 = float(np.median(all_r1))
    C1 = float(np.median(all_c1))
    TAU = R1 * C1
    print("[Success] EKF refinement done.")
except Exception as e:
    print(f'[Error] {e}')
    exit(1)

try:
    print("[Load] Writing ecm_params.h...")
    with open(output_path, 'w') as f:
        f.write("#pragma once\n\n")
        f.write(f"#define ECM_R0_INIT  {R0:.6f}f\n")
        f.write(f"#define ECM_R1_INIT  {R1:.6f}f\n")
        f.write(f"#define ECM_C1_INIT  {C1:.4f}f\n")
        f.write(f"#define ECM_TAU_INIT {TAU:.4f}f\n")
    print("[Success] ecm_params.h saved!")
except Exception as e:
    print(f'[Error] {e}')
    exit(1)

print("========== Done!! ==========")