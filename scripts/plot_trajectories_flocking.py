#!/usr/bin/env python3
"""
Plot robot trajectories for the first 5 minutes (300 s simulation time)
of a flocking FSM experiment.

Output: data/flocking/trajectories_first5min.png
"""

import csv
import math
import os
import re

import numpy as np
import matplotlib.pyplot as plt

plt.rcParams.update({'font.size': 14})

# ── paths ─────────────────────────────────────────────────────────────────
BASE     = os.path.join(os.path.dirname(__file__), '..', 'data', 'flocking')
CSV_PATH = os.path.abspath(os.path.join(BASE, 'flocking_fsm_log.csv'))
LOG_PATH = os.path.abspath(os.path.join(BASE, 'flocking_fsm.txt'))
OUT_PATH = os.path.abspath(os.path.join(BASE, 'trajectories_first5min_tb1_tb2.png'))

ROBOTS      = ['tb1', 'tb2', 'tb3', 'tb4']  # all robots needed for CSV parsing
PLOT_ROBOTS = ['tb1', 'tb2']                # only these two are drawn
COLORS = {'tb1': 'blue', 'tb2': 'red', 'tb3': 'green', 'tb4': 'orange'}

CUTOFF_SIM_S = 300.0   # first 5 minutes

if not os.path.exists(CSV_PATH):
    print(f"Error: {CSV_PATH} does not exist.")
    exit(1)

# ── detect RTF ────────────────────────────────────────────────────────────
RTF = None
if os.path.exists(LOG_PATH):
    with open(LOG_PATH) as f:
        for line in f:
            m = re.search(r'RTF:\s*([\d.]+)', line)
            if m:
                RTF = float(m.group(1))
                break

if RTF and RTF > 0:
    print(f'Detected RTF={RTF:.3f}')
else:
    RTF = 1.0
    print('No RTF found — assuming RTF=1.0 (wall-clock = sim time).')

# ── load CSV ──────────────────────────────────────────────────────────────
timestamps = []
positions  = {r: [] for r in ROBOTS}

with open(CSV_PATH, 'r') as f:
    reader = csv.reader(f)
    next(reader)   # skip header
    for row in reader:
        if len(row) < 9:
            continue
        try:
            ts = float(row[0])
        except ValueError:
            continue

        pts = {}
        valid_all = True
        for i, robot in enumerate(ROBOTS):
            xv, yv = row[1 + 2*i], row[2 + 2*i]
            if xv in ('NaN', 'nan', '') or yv in ('NaN', 'nan', ''):
                valid_all = False
                break
            xf, yf = float(xv), float(yv)
            if not (math.isfinite(xf) and math.isfinite(yf)):
                valid_all = False
                break
            pts[robot] = (float(xv), float(yv))

        if valid_all:
            timestamps.append(ts)
            for robot, p in pts.items():
                positions[robot].append(p)

timestamps = np.array(timestamps)
if len(timestamps) == 0:
    print("No valid rows found.")
    exit(1)

# ── convert to sim time and apply 5-min cutoff ────────────────────────────
t_sim = (timestamps - timestamps[0]) * RTF
mask  = t_sim <= CUTOFF_SIM_S

t_trim = t_sim[mask]
pos    = {r: np.array(positions[r])[mask] for r in ROBOTS}

n_kept = mask.sum()
print(f'Using {n_kept} / {len(timestamps)} samples (sim time 0 – {t_trim[-1]:.1f} s)')

# ── plot ──────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(10, 8))

for robot in PLOT_ROBOTS:
    xy  = pos[robot]
    col = COLORS[robot]

    ax.plot(xy[:, 0], xy[:, 1],
            color=col, linewidth=1.0, alpha=0.8, label=robot)

    # start marker (triangle up)
    ax.scatter(xy[0, 0], xy[0, 1],
               color=col, marker='^', s=120, zorder=5)

    # end marker (triangle down)
    ax.scatter(xy[-1, 0], xy[-1, 1],
               color=col, marker='v', s=120, zorder=5)

ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_title(f'Robot trajectories ({", ".join(PLOT_ROBOTS)}) — first {int(CUTOFF_SIM_S // 60)} min of simulation\n'
             f'(▲ start  ▼ end)')
ax.legend(loc='best')
ax.grid(True, alpha=0.4)
ax.set_aspect('equal')
fig.tight_layout()

os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
fig.savefig(OUT_PATH, dpi=150)
plt.close(fig)
print(f'Saved: {OUT_PATH}')
