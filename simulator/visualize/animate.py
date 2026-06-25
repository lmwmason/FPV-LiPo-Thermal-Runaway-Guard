import csv
import os
import shutil
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from mpl_toolkits.mplot3d import Axes3D

data_path = '../output/simulation.csv'
output_dir = '../output'
os.makedirs(output_dir, exist_ok=True)

FPS = 30
ARM_D = 0.1 * 0.70710678
SIM_DURATION = 20.0

SEGMENTS = [
    (0.0, 2.0, 'Hover', 'gray'),
    (2.0, 4.0, 'Punch-out', 'red'),
    (4.0, 5.0, 'Backflip', 'orange'),
    (5.0, 6.0, 'Recovery', 'gray'),
    (6.0, 8.0, 'Rolls', 'gold'),
    (8.0, 9.0, 'Recovery', 'gray'),
    (9.0, 11.0, 'Power Loop', 'green'),
    (11.0, 13.0, 'Split-S', 'blue'),
    (13.0, 15.0, 'Punch-out Repeats', 'red'),
    (15.0, 18.0, 'Full Throttle Hold', 'red'),
    (18.0, 20.0, 'Land', 'gray'),
]

STATUS_LABELS = {0: ('NORMAL', 'tab:green'), 1: ('WARNING', 'tab:orange'), 2: ('CRITICAL', 'tab:red')}


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
    return {soh: {k: d[k][d['soh'] == soh] for k in d} for soh in order}


def maneuver_at(t):
    for start, end, name, color in SEGMENTS:
        if start <= t < end:
            return name
    return SEGMENTS[-1][2]


def rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    r = np.radians(roll_deg)
    p = np.radians(pitch_deg)
    y = np.radians(yaw_deg)
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_limits(x, y, z, pad_frac=0.1):
    span = max(x.max() - x.min(), y.max() - y.min(), z.max() - z.min(), 1e-3)
    pad = span * pad_frac
    cx, cy, cz = (x.max() + x.min()) / 2, (y.max() + y.min()) / 2, (z.max() + z.min()) / 2
    half = span / 2 + pad
    return (cx - half, cx + half), (cy - half, cy + half), (cz - half, cz + half), span


def main():
    d = load_csv(data_path)
    scenarios = split_by_soh(d)
    sohs = list(scenarios.keys())
    for soh in sohs:
        mask = scenarios[soh]['time'] < SIM_DURATION
        scenarios[soh] = {k: scenarios[soh][k][mask] for k in scenarios[soh]}
    path = scenarios[sohs[0]]

    n_total = len(path['time'])
    n_frames = max(2, int(round((path['time'][-1] - path['time'][0]) * FPS)))
    frame_idx = np.linspace(0, n_total - 1, n_frames).astype(int)

    xlim, ylim, zlim, span = axis_limits(path['x'], path['y'], path['z'])
    icon_scale = span * 0.04

    fig = plt.figure(figsize=(13, 7))
    ax3d = fig.add_subplot(1, 2, 1, projection='3d')
    ax_panel = fig.add_subplot(1, 2, 2)
    ax_panel.axis('off')

    ax3d.set_xlim(*xlim)
    ax3d.set_ylim(*ylim)
    ax3d.set_zlim(*zlim)
    ax3d.set_box_aspect((1, 1, 1))
    ax3d.set_xlabel('X (m)')
    ax3d.set_ylabel('Y (m)')
    ax3d.set_zlabel('Z (m, up)')
    ax3d.set_title('FPV Freestyle Flight Path')

    for start, end, name, color in SEGMENTS:
        mask = (path['time'] >= start) & (path['time'] < end)
        if np.any(mask):
            ax3d.plot(path['x'][mask], path['y'][mask], path['z'][mask],
                      color=color, alpha=0.35, linewidth=1.5)

    marker, = ax3d.plot([], [], [], 'o', color='black', markersize=6, markeredgecolor='white')
    arm_lines = [ax3d.plot([], [], [], color='dimgray', linewidth=2.0)[0] for _ in range(4)]
    quiver_artists = []

    text_y0 = 0.95
    text_time = ax_panel.text(0.02, text_y0, '', fontsize=13, family='monospace', transform=ax_panel.transAxes)
    text_maneuver = ax_panel.text(0.02, text_y0 - 0.06, '', fontsize=13, family='monospace',
                                   transform=ax_panel.transAxes)
    scenario_texts = []
    for i, soh in enumerate(sohs):
        y = text_y0 - 0.18 - i * 0.30
        header = ax_panel.text(0.02, y, f'SOH={soh:.2f}', fontsize=13, family='monospace',
                                weight='bold', transform=ax_panel.transAxes)
        cur_t = ax_panel.text(0.04, y - 0.06, '', fontsize=12, family='monospace', transform=ax_panel.transAxes)
        temp_t = ax_panel.text(0.04, y - 0.12, '', fontsize=12, family='monospace', transform=ax_panel.transAxes)
        ttr_t = ax_panel.text(0.04, y - 0.18, '', fontsize=12, family='monospace', transform=ax_panel.transAxes)
        status_t = ax_panel.text(0.04, y - 0.24, '', fontsize=12, family='monospace', weight='bold',
                                  transform=ax_panel.transAxes)
        scenario_texts.append((cur_t, temp_t, ttr_t, status_t))

    def format_ttr(v):
        return f'{v:.2e}' if v > 9999 else f'{v:.1f}'

    def update(frame_num):
        idx = frame_idx[frame_num]
        t = path['time'][idx]
        x, y, z = path['x'][idx], path['y'][idx], path['z'][idx]
        roll, pitch, yaw = path['roll'][idx], path['pitch'][idx], path['yaw'][idx]
        rot = rotation_matrix(roll, pitch, yaw)

        marker.set_data_3d([x], [y], [z])

        motor_body = [
            (ARM_D, -ARM_D, 0.0),
            (-ARM_D, ARM_D, 0.0),
            (ARM_D, ARM_D, 0.0),
            (-ARM_D, -ARM_D, 0.0),
        ]
        for line, m in zip(arm_lines, motor_body):
            tip = rot @ np.array(m) * (icon_scale / ARM_D if ARM_D > 0 else 1.0)
            line.set_data_3d([x, x + tip[0]], [y, y + tip[1]], [z, z + tip[2]])

        nonlocal quiver_artists
        for q in quiver_artists:
            q.remove()
        quiver_artists = []
        axes_body = [np.array([1, 0, 0]), np.array([0, 0, 1]), np.array([0, -1, 0])]
        axes_color = ['red', 'blue', 'green']
        for vec, color in zip(axes_body, axes_color):
            world = rot @ vec * icon_scale * 1.5
            quiver_artists.append(
                ax3d.quiver(x, y, z, world[0], world[1], world[2], color=color, linewidth=1.5))

        text_time.set_text(f't = {t:5.2f}s')
        text_maneuver.set_text(f'Maneuver: {maneuver_at(t)}')

        for soh, texts in zip(sohs, scenario_texts):
            s = scenarios[soh]
            cur_t, temp_t, ttr_t, status_t = texts
            cur_t.set_text(f'I = {s["current"][idx]:6.2f} A')
            temp_t.set_text(f'T = {s["temperature"][idx]:6.2f} C')
            ttr_t.set_text(f'TTR = {format_ttr(s["time_to_runaway"][idx])} s')
            label, color = STATUS_LABELS[int(s['status'][idx])]
            status_t.set_text(f'Status: {label}')
            status_t.set_color(color)

        return [marker, *arm_lines, *quiver_artists, text_time, text_maneuver]

    anim = FuncAnimation(fig, update, frames=n_frames, interval=1000.0 / FPS, blit=False, repeat=True)
    fig.tight_layout()

    if shutil.which('ffmpeg'):
        out_path = os.path.join(output_dir, 'flight_animation.mp4')
        print(f'[Run] Encoding {out_path} ...')
        writer = FFMpegWriter(fps=FPS)
        anim.save(out_path, writer=writer)
        print(f'[Success] Saved {out_path}')
    else:
        print('[Info] ffmpeg not found, skipping mp4 export.')

    plt.show()


print('[Load] Loading simulation.csv...')
main()
print('========== Done!! ==========')
