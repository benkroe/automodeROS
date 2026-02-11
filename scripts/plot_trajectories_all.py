#!/usr/bin/env python3
"""
Plot full-run robot trajectories for FSM, BT and Exploration experiments.

Outputs:
  data/flocking/trajectories_fsm.png
  data/flocking/trajectories_bt.png
  data/flocking/trajectories_exp.png
"""

import csv
import math
import os
import re

import numpy as np
import matplotlib.pyplot as plt

plt.rcParams.update({'font.size': 28})

BASE   = os.path.join(os.path.dirname(__file__), '..', 'data', 'flocking')
ROBOTS = ['tb1', 'tb2', 'tb3', 'tb4']
COLORS = {'tb1': 'blue', 'tb2': 'red', 'tb3': 'green', 'tb4': 'orange'}

MODES = [
    ('fsm', 'FSM Flocking'),
    ('bt',  'BT Flocking'),
    ('exp', 'Exploration baseline'),
]


def detect_rtf(log_path):
    if not os.path.exists(log_path):
        return 1.0
    with open(log_path) as f:
        for line in f:
            m = re.search(r'RTF:\s*([\d.]+)', line)
            if m:
                return float(m.group(1))
    return 1.0


def load_positions(csv_path, log_path):
    RTF = detect_rtf(log_path)
    timestamps, positions = [], {r: [] for r in ROBOTS}

    with open(csv_path) as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            if len(row) < 9:
                continue
            try:
                ts = float(row[0])
            except ValueError:
                continue
            pts, ok = {}, True
            for i, robot in enumerate(ROBOTS):
                xv, yv = row[1 + 2*i], row[2 + 2*i]
                if xv in ('NaN', 'nan', '') or yv in ('NaN', 'nan', ''):
                    ok = False; break
                xf, yf = float(xv), float(yv)
                if not (math.isfinite(xf) and math.isfinite(yf)):
                    ok = False; break
                pts[robot] = (xf, yf)
            if ok:
                timestamps.append(ts)
                for robot, p in pts.items():
                    positions[robot].append(p)

    if not timestamps:
        return None, None, RTF

    # Detect already-sim-time timestamps
    if timestamps[0] < 1e6:
        RTF = 1.0

    pos = {r: np.array(positions[r]) for r in ROBOTS}
    return pos, np.array(timestamps), RTF


os.makedirs(os.path.abspath(BASE), exist_ok=True)

for mode, title in MODES:
    csv_path = os.path.abspath(os.path.join(BASE, f'flocking_{mode}_log.csv'))
    log_path = os.path.abspath(os.path.join(BASE, f'flocking_{mode}.txt'))
    out_path = os.path.abspath(os.path.join(BASE, f'trajectories_{mode}.png'))

    if not os.path.exists(csv_path):
        print(f'Skipping {mode}: {csv_path} not found')
        continue

    pos, timestamps, RTF = load_positions(csv_path, log_path)
    if pos is None:
        print(f'No valid data for {mode}')
        continue

    print(f'[{mode}] RTF={RTF:.3f}  samples={len(timestamps)}')

    fig, ax = plt.subplots(figsize=(10, 8))

    for robot in ROBOTS:
        xy = pos[robot]
        ax.plot(xy[:, 0], xy[:, 1],
                color=COLORS[robot], marker='o', markersize=2,
                linewidth=0.8, alpha=0.85,
                label=f'{robot} trajectory')

    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_title(f'Robot Trajectories ({title})')
    ax.legend(loc='lower right', markerscale=4)
    ax.grid(True)
    ax.set_aspect('equal')
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f'Saved: {out_path}')
