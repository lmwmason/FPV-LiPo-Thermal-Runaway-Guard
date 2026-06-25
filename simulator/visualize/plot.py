import csv
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

data_path = '../output/simulation.csv'
output_dir = './output'
os.makedirs(output_dir, exist_ok=True)

plt.rcParams['figure.dpi'] = 150
plt.rcParams['savefig.dpi'] = 300
plt.rcParams['font.size'] = 10

TIME_WARNING = 300.0
TIME_CRITICAL = 100.0
TTR_CAP = 600.0

STATUS_LABELS = {0: 'NORMAL', 1: 'WARNING', 2: 'CRITICAL'}
SCENARIO_COLORS = ['tab:blue', 'tab:red', 'tab:green', 'tab:purple']


def load_csv(path):
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    columns = {}
    for key in reader.fieldnames:
        if key == 'status':
            columns[key] = np.array([int(row[key]) for row in rows])
        else:
            columns[key] = np.array([float(row[key]) for row in rows])
    return columns


def split_by_soh(d):
    order = []
    for v in d['soh']:
        if v not in order:
            order.append(v)
    return [(soh, {k: d[k][d['soh'] == soh] for k in d}) for soh in order]


def plot_throttle_current(ax, scenarios):
    ax2 = ax.twinx()
    for (soh, s), color in zip(scenarios, SCENARIO_COLORS):
        label = f'SOH={soh:.2f}'
        ax.plot(s['time'], s['throttle'], color=color, linestyle='--', linewidth=1.0, alpha=0.6,
                 label=f'{label} throttle')
        ax2.plot(s['time'], s['current'], color=color, linewidth=1.4,
                  label=f'{label} current')
    ax.set_ylabel('Throttle')
    ax2.set_ylabel('Current (A)')
    ax.set_title('Throttle & Current vs Time')
    ax.grid(alpha=0.3)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc='upper right', fontsize=8)


def plot_voltage_soc(ax, scenarios):
    ax2 = ax.twinx()
    for (soh, s), color in zip(scenarios, SCENARIO_COLORS):
        label = f'SOH={soh:.2f}'
        ax.plot(s['time'], s['voltage'], color=color, linewidth=1.4,
                 label=f'{label} voltage')
        ax2.plot(s['time'], s['soc'], color=color, linestyle='--', linewidth=1.2, alpha=0.7,
                  label=f'{label} SOC')
    ax.set_ylabel('Voltage (V)')
    ax2.set_ylabel('SOC')
    ax.set_title('Voltage & SOC vs Time')
    ax.grid(alpha=0.3)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc='lower left', fontsize=8)


def plot_temperature(ax, scenarios):
    for (soh, s), color in zip(scenarios, SCENARIO_COLORS):
        ax.plot(s['time'], s['temperature'], color=color, linewidth=1.4, label=f'SOH={soh:.2f}')
    ax.set_ylabel('Temperature (C)')
    ax.set_title('Battery Temperature vs Time')
    ax.grid(alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)


def plot_time_to_runaway(ax, scenarios):
    for (soh, s), color in zip(scenarios, SCENARIO_COLORS):
        ttr = np.minimum(s['time_to_runaway'], TTR_CAP)
        ax.plot(s['time'], ttr, color=color, linewidth=1.4, label=f'SOH={soh:.2f}')
    ax.axhline(TIME_WARNING, color='tab:orange', linestyle=':', linewidth=1.0, label='Warning (300s)')
    ax.axhline(TIME_CRITICAL, color='tab:red', linestyle=':', linewidth=1.0, label='Critical (100s)')
    ax.set_ylabel(f'Time to Runaway (s, capped at {TTR_CAP:.0f})')
    ax.set_title('Thermal Runaway Prediction vs Time')
    ax.grid(alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)


def plot_controller(ax, scenarios):
    ax2 = ax.twinx()
    for (soh, s), color in zip(scenarios, SCENARIO_COLORS):
        label = f'SOH={soh:.2f}'
        ax.plot(s['time'], s['output_limit'], color=color, linewidth=1.4,
                 label=f'{label} output_limit')
        ax2.step(s['time'], s['status'], where='post', color=color, linestyle=':', linewidth=1.0, alpha=0.7,
                  label=f'{label} status')
    ax.set_ylabel('Output Limit')
    ax2.set_ylabel('Status')
    ax2.set_yticks([0, 1, 2])
    ax2.set_yticklabels([STATUS_LABELS[i] for i in range(3)])
    ax.set_xlabel('Time (s)')
    ax.set_title('Controller Behavior: Output Limit & Status vs Time')
    ax.grid(alpha=0.3)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc='lower left', fontsize=8)


def main():
    d = load_csv(data_path)
    scenarios = split_by_soh(d)

    fig, axes = plt.subplots(5, 1, figsize=(11, 19), sharex=True)

    plot_throttle_current(axes[0], scenarios)
    plot_voltage_soc(axes[1], scenarios)
    plot_temperature(axes[2], scenarios)
    plot_time_to_runaway(axes[3], scenarios)
    plot_controller(axes[4], scenarios)

    fig.suptitle('FPV Freestyle Flight: Battery Thermal Runaway Guard Simulation')
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(os.path.join(output_dir, 'simulation.png'))
    plt.close(fig)

    for soh, s in scenarios:
        print(f"[Info] SOH={soh:.2f}: peak current={s['current'].max():.2f}A, "
              f"peak temp={s['temperature'].max():.1f}C, "
              f"min time_to_runaway={s['time_to_runaway'].min():.1f}s")


print("[Load] Loading simulation.csv...")
main()
print("========== Done!! Saved figure in ./output directory! ==========")
