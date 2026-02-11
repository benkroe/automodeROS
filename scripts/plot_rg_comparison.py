#!/usr/bin/env python3
"""
Compare radius of gyration (Rg) across FSM, BT and Exploration experiments.

Rg(t) = sqrt( (1/N) * sum_i ||r_i(t) - r_centroid(t)||^2 )

Lower Rg = robots more compact around their common centroid.

Outputs:
  data/flocking/rg_comparison.png   – Rg timeseries for all three modes
"""

import csv
import math
import os
import re

import numpy as np
import matplotlib.pyplot as plt

plt.rcParams.update({'font.size': 14})

BASE    = os.path.join(os.path.dirname(__file__), '..', 'data', 'flocking')
OUT_DIR = os.path.abspath(BASE)
ROBOTS  = ['tb1', 'tb2', 'tb3', 'tb4']

MODES = [
    ('fsm', 'FSM flocking',         'royalblue'),
    ('bt',  'BT flocking',          'darkorange'),
    ('exp', 'Exploration baseline', 'grey'),
]


# ── helpers ───────────────────────────────────────────────────────────────
def detect_rtf(log_path):
    if not os.path.exists(log_path):
        return 1.0
    with open(log_path) as f:
        for line in f:
            m = re.search(r'RTF:\s*([\d.]+)', line)
            if m:
                return float(m.group(1))
    return 1.0


def load_rg(mode):
    csv_path = os.path.abspath(os.path.join(BASE, f'flocking_{mode}_log.csv'))
    log_path = os.path.abspath(os.path.join(BASE, f'flocking_{mode}.txt'))

    if not os.path.exists(csv_path):
        raise FileNotFoundError(csv_path)

    RTF = detect_rtf(log_path)
    print(f'[{mode}] RTF={RTF:.3f}')

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
        raise ValueError(f'No valid rows in {csv_path}')

    # If timestamps are already sim-time (< 1e6), don't apply RTF
    if timestamps[0] < 1e6:
        RTF = 1.0

    t   = (np.array(timestamps) - timestamps[0]) * RTF
    pos = {r: np.array(positions[r]) for r in ROBOTS}    # (N, 2)

    centroid = np.mean([pos[r] for r in ROBOTS], axis=0)  # (N, 2)
    rg = np.sqrt(
        np.mean([np.sum((pos[r] - centroid)**2, axis=1) for r in ROBOTS], axis=0)
    )   # (N,)

    return t, rg


# ── load all ──────────────────────────────────────────────────────────────
results = {}
for mode, label, color in MODES:
    try:
        t, rg = load_rg(mode)
        results[mode] = (t, rg, label, color)
        print(f'  → mean Rg = {np.mean(rg):.3f} m  duration = {t[-1]:.1f} s  ({len(t)} samples)')
    except FileNotFoundError as e:
        print(f'Skipping {mode}: {e}')

# ── plot: Rg timeseries ───────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(13, 5))

for mode, (t, rg, label, color) in results.items():
    mean_rg = np.mean(rg)
    ax.plot(t, rg, color=color, linewidth=1.2, alpha=0.85, label=f'{label}  (mean={mean_rg:.2f} m)')
    ax.axhline(mean_rg, color=color, linestyle='--', linewidth=1.0, alpha=0.6)

ax.set_xlabel('Simulation time (s)')
ax.set_ylabel('Radius of gyration $R_g$ (m)')
ax.set_title('Group compactness over time — Radius of gyration')
ax.legend(fontsize=11)
ax.grid(True, alpha=0.35)
fig.tight_layout()

out = os.path.join(OUT_DIR, 'rg_comparison.png')
os.makedirs(OUT_DIR, exist_ok=True)
fig.savefig(out, dpi=150)
plt.close(fig)
print(f'\nSaved: {out}')
