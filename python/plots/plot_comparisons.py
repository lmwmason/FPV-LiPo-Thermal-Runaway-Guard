import csv
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

data_dir = '../../c/build/data'
output_dir = './output'
os.makedirs(output_dir, exist_ok=True)

plt.rcParams['figure.dpi'] = 150
plt.rcParams['savefig.dpi'] = 300
plt.rcParams['font.size'] = 10


def load_csv(name):
    path = os.path.join(data_dir, name)
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    columns = {}
    for key in reader.fieldnames:
        columns[key] = np.array([float(row[key]) for row in rows])
    return columns


def per_cycle_rmse(cycle, measured, predicted):
    cycles = np.unique(cycle).astype(int)
    rmse = np.empty(len(cycles))
    for idx, c in enumerate(cycles):
        mask = cycle == c
        rmse[idx] = np.sqrt(np.mean((measured[mask] - predicted[mask]) ** 2))
    return cycles, rmse


def plot_compare_a():
    d = load_csv('compare_a.csv')
    cycles, rmse_1rc = per_cycle_rmse(d['cycle'], d['voltage_measured'], d['voltage_1rc'])
    _, rmse_2rc = per_cycle_rmse(d['cycle'], d['voltage_measured'], d['voltage_2rc'])

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))

    axes[0].plot(cycles, rmse_1rc, label='1RC', color='tab:orange', linewidth=1.4)
    axes[0].plot(cycles, rmse_2rc, label='2RC', color='tab:blue', linewidth=1.4)
    axes[0].set_xlabel('Cycle')
    axes[0].set_ylabel('Voltage RMSE (V)')
    axes[0].set_title('Per-cycle voltage RMSE: 1RC vs 2RC')
    axes[0].legend()
    axes[0].grid(alpha=0.3)

    sample_cycle = 50
    mask = d['cycle'] == sample_cycle
    axes[1].plot(d['time'][mask], d['voltage_measured'][mask], label='Measured', color='black', linewidth=1.6)
    axes[1].plot(d['time'][mask], d['voltage_1rc'][mask], label='1RC predicted', color='tab:orange', linestyle='--')
    axes[1].plot(d['time'][mask], d['voltage_2rc'][mask], label='2RC predicted', color='tab:blue', linestyle='--')
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Voltage (V)')
    axes[1].set_title(f'Voltage trace, cycle {sample_cycle}')
    axes[1].legend()
    axes[1].grid(alpha=0.3)

    fig.suptitle('Comparison A: 1RC vs 2RC ECM voltage prediction')
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'compare_a_ecm_order.png'))
    plt.close(fig)
    print(f"[Info] mean RMSE 1RC={np.mean(rmse_1rc):.4f}V, 2RC={np.mean(rmse_2rc):.4f}V")


def plot_compare_b():
    d = load_csv('compare_b.csv')
    cycles, rmse_coulomb = per_cycle_rmse(d['cycle'], d['soc_true'], d['soc_coulomb'])
    _, rmse_ekf = per_cycle_rmse(d['cycle'], d['soc_true'], d['soc_ekf'])

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))

    axes[0].plot(cycles, rmse_coulomb, label='Coulomb counting', color='tab:orange', linewidth=1.4)
    axes[0].plot(cycles, rmse_ekf, label='EKF', color='tab:blue', linewidth=1.4)
    axes[0].set_xlabel('Cycle')
    axes[0].set_ylabel('SOC RMSE')
    axes[0].set_title('Per-cycle SOC RMSE')
    axes[0].legend()
    axes[0].grid(alpha=0.3)

    for ax, sample_cycle in zip(axes[1:], [0, 167]):
        mask = d['cycle'] == sample_cycle
        ax.plot(d['time'][mask], d['soc_true'][mask], label='True SOC', color='black', linewidth=1.6)
        ax.plot(d['time'][mask], d['soc_coulomb'][mask], label='Coulomb counting', color='tab:orange', linestyle='--')
        ax.plot(d['time'][mask], d['soc_ekf'][mask], label='EKF', color='tab:blue', linestyle='--')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('SOC')
        ax.set_title(f'SOC trace, cycle {sample_cycle}')
        ax.legend()
        ax.grid(alpha=0.3)

    fig.suptitle('Comparison B: Coulomb counting vs EKF SOC estimation')
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'compare_b_soc_method.png'))
    plt.close(fig)
    print(f"[Info] mean RMSE coulomb={np.mean(rmse_coulomb):.4f}, ekf={np.mean(rmse_ekf):.4f}")


def plot_compare_c():
    d = load_csv('compare_c.csv')

    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(d['cycle'], d['soh_r0'], label='R0-based SOH', color='tab:blue', linewidth=1.6)
    ax.plot(d['cycle'], d['soh_capacity'], label='Capacity-based SOH', color='tab:green', linewidth=1.6)
    ax.axhline(0.8, color='tab:orange', linestyle=':', linewidth=1.0, label='SOH warning (0.8)')
    ax.axhline(0.6, color='tab:red', linestyle=':', linewidth=1.0, label='SOH critical (0.6)')
    ax.set_xlabel('Cycle')
    ax.set_ylabel('SOH')
    ax.set_title('Comparison C: R0-based vs capacity-based SOH')
    ax.legend()
    ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'compare_c_soh_method.png'))
    plt.close(fig)


def plot_compare_d():
    d = load_csv('compare_d.csv')
    cycle = d['cycle']
    simple_alert = d['simple_alert_time'] >= 0
    arrhenius_alert = d['arrhenius_alert_time'] >= 0

    fig, axes = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)

    axes[0].step(cycle, simple_alert.astype(int), where='post', label='Temperature threshold (40C)', color='tab:orange')
    axes[0].step(cycle, arrhenius_alert.astype(int) - 0.02, where='post', label='Arrhenius (relative)', color='tab:blue')
    axes[0].set_ylabel('Alert state')
    axes[0].set_yticks([0, 1])
    axes[0].set_yticklabels(['OFF', 'ON'])
    axes[0].set_title('Per-cycle alert state: flutter vs persistence')
    axes[0].legend(loc='center right')
    axes[0].grid(alpha=0.3)

    axes[1].scatter(cycle[simple_alert], d['simple_alert_time'][simple_alert],
                     label='Temperature threshold (40C)', color='tab:orange', s=14)
    axes[1].scatter(cycle[arrhenius_alert], d['arrhenius_alert_time'][arrhenius_alert],
                     label='Arrhenius (relative)', color='tab:blue', s=14)
    axes[1].set_xlabel('Cycle')
    axes[1].set_ylabel('Alert time within cycle (s)')
    axes[1].set_title('Alert timing within cycle')
    axes[1].legend()
    axes[1].grid(alpha=0.3)

    fig.suptitle('Comparison D: Temperature threshold vs Arrhenius-based alert')
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'compare_d_runaway_alert.png'))
    plt.close(fig)
    print(f"[Info] simple alerts: {int(simple_alert.sum())}/{len(cycle)} cycles, "
          f"arrhenius alerts: {int(arrhenius_alert.sum())}/{len(cycle)} cycles")


def plot_compare_e():
    d = load_csv('compare_e.csv')

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))

    axes[0].plot(d['cycle'], d['output_limit_avg'], color='tab:purple', linewidth=1.4)
    axes[0].set_xlabel('Cycle')
    axes[0].set_ylabel('Average output limit')
    axes[0].set_title('Adaptive control output limit per cycle')
    axes[0].grid(alpha=0.3)

    axes[1].plot(d['cycle'], d['cum_risk_time_fixed'], label='Fixed 100% output', color='tab:red', linewidth=1.6)
    axes[1].plot(d['cycle'], d['cum_risk_time_adaptive'], label='Adaptive control', color='tab:green', linewidth=1.6)
    axes[1].set_xlabel('Cycle')
    axes[1].set_ylabel('Cumulative risk exposure time (s)')
    axes[1].set_title('Cumulative time spent in WARNING/CRITICAL state')
    axes[1].legend()
    axes[1].grid(alpha=0.3)

    fig.suptitle('Comparison E: Fixed output vs adaptive power control')
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, 'compare_e_power_control.png'))
    plt.close(fig)
    final_fixed = d['cum_risk_time_fixed'][-1]
    final_adaptive = d['cum_risk_time_adaptive'][-1]
    reduction = (final_fixed - final_adaptive) / final_fixed * 100.0
    print(f"[Info] total risk exposure: fixed={final_fixed:.1f}s, adaptive={final_adaptive:.1f}s, "
          f"reduction={reduction:.1f}%")


print("[Load] Plotting comparison A...")
plot_compare_a()
print("[Load] Plotting comparison B...")
plot_compare_b()
print("[Load] Plotting comparison C...")
plot_compare_c()
print("[Load] Plotting comparison D...")
plot_compare_d()
print("[Load] Plotting comparison E...")
plot_compare_e()
print("========== Done!! Saved figures in ./output directory! ==========")
