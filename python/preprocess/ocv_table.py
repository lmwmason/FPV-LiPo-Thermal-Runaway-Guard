import csv
import os
import numpy as np

input_dir = './output/discharge'
output_path = './output/ocv_table.h'
Q_NOMINAL = 2.0 * 3600.0
MAX_CYCLE = 10
V_CUTOFF = 2.5

ECM_R0 = 0.078491
ECM_R1 = 0.036401
ECM_C1 = 2639.5083

cycles = {}
try:
    print("[Load] Loading discharge CSVs...")
    battery_files = sorted([f for f in os.listdir(input_dir) if f.endswith('.csv')])
    for battery_file in battery_files:
        with open(os.path.join(input_dir, battery_file), 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                cycle_id_num = int(row['cycle'])
                if cycle_id_num > MAX_CYCLE:
                    continue
                cycle_id = f"{battery_file}_{row['cycle']}"
                time = float(row['time'])
                voltage = float(row['voltage'])
                current = float(row['current'])
                if cycle_id not in cycles:
                    cycles[cycle_id] = []
                cycles[cycle_id].append((time, voltage, current))
    print(f"[Success] Loaded {len(cycles)} cycles from {len(battery_files)} batteries.")
except Exception as e:
    print(f'[Error] {e}')
    exit(1)

try:
    print("[Load] Extracting IR/polarization-compensated OCV-SOC table...")
    ocv_points = []
    for cycle_id in sorted(cycles.keys()):
        samples = cycles[cycle_id]
        soc = 1.0
        v_rc = 0.0
        for idx, (time, voltage, current) in enumerate(samples):
            if voltage < V_CUTOFF:
                break
            if idx == 0:
                dt = 0.0
            else:
                dt = time - samples[idx - 1][0]
            i_abs = abs(current)
            soc -= (i_abs * dt) / Q_NOMINAL
            soc = max(0.0, min(1.0, soc))

            tau = ECM_R1 * ECM_C1
            exp_tau = np.exp(-dt / tau) if tau > 0.0 else 0.0
            v_rc = v_rc * exp_tau + i_abs * ECM_R1 * (1.0 - exp_tau)

            v_oc_est = voltage + i_abs * ECM_R0 + v_rc
            ocv_points.append((soc, v_oc_est))

    ocv_points.sort(key=lambda x: x[0])
    soc_steps = np.linspace(0.0, 1.0, 11)
    ocv_table = []
    for soc_target in soc_steps:
        soc_arr = np.array([p[0] for p in ocv_points])
        ocv_arr = np.array([p[1] for p in ocv_points])
        ocv_interp = np.interp(soc_target, soc_arr, ocv_arr)
        ocv_table.append((soc_target, ocv_interp))
    print(f"[Success] Extracted {len(ocv_table)} OCV points.")
except Exception as e:
    print(f'[Error] {e}')
    exit(1)

try:
    print("[Load] Writing ocv_table.h...")
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w') as f:
        f.write("#pragma once\n\n")
        f.write("#define OCV_TABLE_SIZE 11\n\n")
        f.write("static const float OCV_SOC_TABLE[OCV_TABLE_SIZE] = {\n")
        for soc, ocv in ocv_table:
            f.write(f"    {ocv:.4f}f,  // SOC {soc*100:.0f}%\n")
        f.write("};\n")
    print("[Success] ocv_table.h saved!")
except Exception as e:
    print(f'[Error] {e}')
    exit(1)

print("========== Done!! ==========")
