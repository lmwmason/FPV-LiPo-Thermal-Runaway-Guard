# FPV-LiPo-Thermal-Runaway-Guard

> **EKF-Based SOH Estimation and Adaptive Output Control for Early Thermal Runaway Prediction in LiPo Batteries**

A software-based thermal runaway prediction system for FPV drone LiPo batteries, combining an Equivalent Circuit Model (ECM) with an Extended Kalman Filter (EKF) for real-time SOH estimation and Arrhenius reaction kinetics for thermal runaway time prediction.

[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)
[![C](https://img.shields.io/badge/C-C11-A8B9CC)](https://en.wikipedia.org/wiki/C11_(C_standard_revision))
[![Python](https://img.shields.io/badge/Python-3.10+-3776AB)](https://www.python.org/)

---

## Overview

FPV drones use standard LiPo batteries without smart battery systems, making it impossible to monitor battery degradation state during operation. Combined with the high-current pulse patterns of freestyle and tactical FPV flight, this creates an undetected thermal runaway risk.

**FPV-LiPo-Thermal-Runaway-Guard** addresses this by:

1. Estimating SOC and SOH in real time using a 1RC ECM + 5-state EKF from voltage, current, and temperature measurements alone
2. Predicting time to thermal runaway using Arrhenius SEI decomposition kinetics
3. Applying adaptive output limiting (NORMAL / WARNING / CRITICAL) based on predicted risk
4. Validating the system against the NASA PCoE dataset through five comparative experiments
5. Running an FPV freestyle endurance simulation to demonstrate degradation-dependent risk acceleration

---

## Reports

The full technical reports are available in both languages:

- [**English Report**](report/Report_EN.pdf)
- [**한국어 보고서**](report/Report_KR.pdf)

---

## Repository Structure

```
FPV-LiPo-Thermal-Runaway-Guard/
├── python/                     # Data preprocessing and visualization
│   ├── preprocess/             # NASA .mat → CSV, OCV table, ECM params
│   │   ├── main.py             # Discharge CSV extraction
│   │   ├── ocv_table.py        # OCV-SOC lookup table extraction
│   │   └── ecm_params.py       # ECM parameter extraction + EKF refinement
│   ├── visualize/              # Comparison experiment plots
│   │   └── plot_comparisons.py
│   ├── dataset/                # NASA PCoE .mat files (not committed)
│   └── output/                 # Generated headers and CSVs
│       ├── ocv_table.h
│       ├── ecm_params.h
│       └── discharge/
├── c/                          # Core algorithm (C11)
│   ├── include/
│   │   ├── ocv_table.h         # OCV-SOC lookup table
│   │   ├── ecm_params.h        # ECM initial parameters
│   │   ├── ecm.h / ecm.c       # 1RC ECM model
│   │   ├── ekf.h / ekf.c       # 5-state EKF
│   │   ├── arrhenius.h / arrhenius.c  # Thermal runaway prediction
│   │   └── controller.h / controller.c  # Adaptive output limiter
│   ├── src/
│   ├── data/                   # Discharge CSVs for validation
│   ├── compare/                # Five comparison experiment executables
│   ├── main.c                  # Full pipeline, outputs output.csv
│   └── CMakeLists.txt
└── simulator/                  # FPV freestyle endurance simulator
    ├── src/
    │   ├── main.c              # Simulation entry point
    │   ├── physics.c           # 12-state quadrotor RK4 physics
    │   ├── acro.c              # Acro rate controller
    │   ├── trajectory.c        # Hardcoded freestyle maneuver sequence
    │   └── battery_sim.c       # Battery bridge (ECM + EKF + Arrhenius)
    ├── visualize/
    │   ├── plot.py             # Simulation result plots
    │   └── animate.py          # 3D flight path animation
    └── CMakeLists.txt
```

---

## How It Works

```
① Download NASA PCoE dataset (.mat files)
② Run python/preprocess/ scripts → generates ocv_table.h, ecm_params.h, discharge CSVs
③ Build c/ with CMake → run main executable → outputs output.csv
④ Run python/visualize/ → generates comparison plots (A–E)
⑤ Build simulator/ with CMake → run endurance simulation
⑥ Run simulator/visualize/plot.py and animate.py → simulation plots + 3D animation
```

---

## System Pipeline

```
Inputs: V(t), I(t), T(t)
        ↓
  1RC ECM Model
  V = OCV(SOC) - I·R0 - V_RC
        ↓
  5-State EKF
  x = [SOC, V_RC, R0, R1, C1]
        ↓
  SOH = R0_init / R0_current
        ↓
  Arrhenius Prediction
  k = A·exp(-Ea/RT)
  t_runaway = (Q_th - Q_SEI) / k·(1 + α·(1 - SOH))
        ↓
  Adaptive Controller
  NORMAL → 100% / WARNING → 70% / CRITICAL → 0%
```

---

## Comparative Experiments

Five experiments were conducted on the NASA PCoE dataset (B0005–B0018, 168 cycles):

| Experiment | Comparison | Result |
|---|---|---|
| A | 1RC vs 2RC ECM | 2RC 6% better RMSE; 1RC chosen for embedded practicality |
| B | Coulomb counting vs EKF SOC | EKF 22% lower RMSE |
| C | R0-based vs capacity-based SOH | R0-based more sensitive; correlation 0.82 |
| D | Temperature threshold vs Arrhenius alert | Arrhenius stable persistent alert vs temperature chattering |
| E | Fixed output vs adaptive control | 34% reduction in cumulative risk exposure time |

---

## FPV Freestyle Endurance Simulation

A physics-based endurance simulator repeatedly executes a 20-second FPV freestyle sequence (punch-out, backflip, successive rolls, power loop, split-S) and runs the full battery monitoring pipeline.

| Scenario | Time to CRITICAL |
|---|---|
| SOH = 1.00 (new battery) | ~10.5 days |
| SOH = 0.70 (degraded battery) | ~4.0 days |

Degraded battery reaches critical state **2.6× faster** than a new battery.

### Simulator Specs (5-inch FPV)

| Parameter | Value |
|---|---|
| Mass | 0.35 kg |
| Max thrust | 40 N (4 × 10 N) |
| Max rate | Roll/Pitch 720°/s, Yaw 360°/s |
| Physics | 12-state RK4, dt = 1 ms |
| Battery current model | I = throttle² × 40A |
| Temperature model | Joule heating approximation |

---

## ECM Parameters

Extracted from NASA PCoE B0005–B0018 dataset (initial 10 cycles, EKF-refined):

| Parameter | Value |
|---|---|
| R0 | 0.0785 Ω |
| R1 | 0.0364 Ω |
| C1 | 2639.5 F |
| τ = R1·C1 | 96.1 s |

---

## Adaptive Controller

| State | Condition | Output Limit |
|---|---|---|
| NORMAL | t_runaway > 300s and SOH > 0.8 | 100% |
| WARNING | 100s < t_runaway ≤ 300s or SOH ≤ 0.8 | 70% |
| CRITICAL | t_runaway ≤ 100s or SOH ≤ 0.6 | 0% |

---

## Build

### Prerequisites
- CMake ≥ 3.20
- GCC with C11 support
- Python 3.10+ with `scipy`, `numpy`, `matplotlib`

### C Core

```bash
cd c
mkdir build && cd build
cmake ..
make
./FPV_LiPo_Thermal_Runaway_Guard
```

### Simulator

```bash
cd simulator
mkdir build && cd build
cmake ..
make
./fpv_battery_sim
```

### Python Preprocessing

```bash
cd python
pip install -r requirements.txt
python preprocess/main.py
python preprocess/ocv_table.py
python preprocess/ecm_params.py
```

---

## Dataset

NASA PCoE Battery Dataset (B0005, B0006, B0007, B0018) must be downloaded separately:

```
https://phm-datasets.s3.amazonaws.com/NASA/5.+Battery+Data+Set.zip
```

Place `.mat` files in `python/dataset/`.

## License

Licensed under the [Apache License 2.0](LICENSE).
