#!/usr/bin/env python3
"""
Analyze flocking position log (FSM or BT).

Usage:
  python3 analyze_flocking.py           # defaults to FSM
  python3 analyze_flocking.py --mode bt # use BT log files

Metrics:
  - All pairwise inter-robot distances over time  → pairwise_distances[_bt].png
  - Mean pairwise distance + group spread (std)   → cohesion[_bt].png
  - Per-robot distance from group centroid        → centroid_deviation[_bt].png
"""

import argparse
import csv
import math
import itertools
import os

import numpy as np
import matplotlib.pyplot as plt

plt.rcParams.update({'font.size': 28})

# ── CLI ───────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument('--mode', choices=['fsm', 'bt', 'exp'], default='fsm',
                    help='Which experiment type to analyse (default: fsm)')
args = parser.parse_args()
MODE   = args.mode
SUFFIX = f'_{MODE}' if MODE != 'fsm' else ''
LABEL  = {'fsm': 'FSM', 'bt': 'BT', 'exp': 'Exploration baseline'}[MODE]

# ── paths ─────────────────────────────────────────────────────────────────
BASE = os.path.join(os.path.dirname(__file__), '..', 'data', 'flocking')
CSV_PATH  = os.path.abspath(os.path.join(BASE, f'flocking_{MODE}_log.csv'))
LOG_PATH  = os.path.abspath(os.path.join(BASE, f'flocking_{MODE}.txt'))
OUT_DIR   = os.path.abspath(BASE)

ROBOTS = ['tb1', 'tb2', 'tb3', 'tb4']
COLORS = {'tb1': 'blue', 'tb2': 'red', 'tb3': 'green', 'tb4': 'orange'}

if not os.path.exists(CSV_PATH):
    print(f"Error: {CSV_PATH} does not exist.")
    exit(1)

os.makedirs(OUT_DIR, exist_ok=True)

# ── detect RTF from log file to convert wall-clock → sim-time ─────────────
# If the first timestamp is already small (sim time, not Unix epoch),
# no scaling is needed regardless of what the log says.
import re as _re
RTF = None
if os.path.exists(LOG_PATH):
    with open(LOG_PATH) as _f:
        for _line in _f:
            _m = _re.search(r'RTF:\s*([\d.]+)', _line)
            if _m:
                RTF = float(_m.group(1))
                break
if RTF and RTF > 0:
    TIME_LABEL = 'Simulation time (s)'
    print(f'Detected RTF={RTF:.3f} — converting wall-clock timestamps to sim time.')
else:
    RTF = 1.0
    TIME_LABEL = 'Wall-clock time (s)'
    print('No RTF found in log — plotting wall-clock time.')

# ── load ──────────────────────────────────────────────────────────────────
def load_csv(path):
    timestamps = []
    positions  = {r: [] for r in ROBOTS}

    with open(path, 'r') as f:
        reader = csv.reader(f)
        next(reader)                          # skip header
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
                pts[robot] = (xf, yf)

            if valid_all:
                timestamps.append(ts)
                for robot, p in pts.items():
                    positions[robot].append(p)

    return np.array(timestamps), positions


timestamps, positions = load_csv(CSV_PATH)
if len(timestamps) == 0:
    print("No valid complete rows found."); exit(1)

# If the first timestamp is already small (< 1e6) it is already sim-time
# (position_collector ran with use_sim_time:True). Skip RTF scaling.
if timestamps[0] < 1e6:
    RTF = 1.0
    TIME_LABEL = 'Simulation time (s)'
    print('Timestamps are already in sim-time — no RTF scaling applied.')

# Relative time from first valid sample; scale by RTF to get simulation time
t = (timestamps - timestamps[0]) * RTF

# Build (N, 2) arrays per robot
pos = {r: np.array(positions[r]) for r in ROBOTS}   # shape (N, 2)
N = len(t)


# ── centroid ──────────────────────────────────────────────────────────────
centroid = np.mean([pos[r] for r in ROBOTS], axis=0)   # (N, 2)


# ── metrics ───────────────────────────────────────────────────────────────
pairs      = list(itertools.combinations(ROBOTS, 2))
pair_dists = {}
for a, b in pairs:
    diff = pos[a] - pos[b]                                   # (N, 2)
    pair_dists[(a, b)] = np.linalg.norm(diff, axis=1)        # (N,)

mean_dist  = np.mean([pair_dists[p] for p in pairs], axis=0) # (N,)

# spread = std of all robot positions around centroid at each timestep
spread = np.std([np.linalg.norm(pos[r] - centroid, axis=1) for r in ROBOTS], axis=0)

centroid_dev = {
    r: np.linalg.norm(pos[r] - centroid, axis=1)
    for r in ROBOTS
}


# ── plot 1: pairwise distances ────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(12, 6))
for a, b in pairs:
    ax.plot(t, pair_dists[(a, b)], label=f'{a}–{b}', alpha=0.8)
ax.set_xlabel(TIME_LABEL)
ax.set_ylabel('Distance (m)')
ax.set_title(f'Pairwise inter-robot distances over time ({LABEL})')
ax.legend(ncol=2)
ax.grid(True, alpha=0.4)
fig.tight_layout()
out1 = os.path.join(OUT_DIR, f'pairwise_distances{SUFFIX}.png')
fig.savefig(out1)
plt.close(fig)
print(f"Saved: {out1}")


# ── plot 2: mean pairwise distance + group spread (cohesion) ──────────────
fig, ax = plt.subplots(figsize=(12, 5))
ax.plot(t, mean_dist, color='royalblue', linewidth=2)
overall = np.mean(mean_dist)
ax.axhline(overall, color='royalblue', linestyle='--', linewidth=1.2)
ax.text(0.98, 0.04, f'mean = {overall:.2f} m', color='royalblue',
        transform=ax.transAxes, va='bottom', ha='right',
        fontsize=plt.rcParams['font.size'])
ax.set_xlabel(TIME_LABEL)
ax.set_ylabel('Distance (m)')
ax.set_title(f'Group cohesion over time ({LABEL})')
ax.grid(True, alpha=0.4)
fig.tight_layout()
out2 = os.path.join(OUT_DIR, f'cohesion{SUFFIX}.png')
fig.savefig(out2)
plt.close(fig)
print(f"Saved: {out2}")


# ── plot 3: per-robot distance from centroid ──────────────────────────────
fig, ax = plt.subplots(figsize=(12, 5))
for r in ROBOTS:
    ax.plot(t, centroid_dev[r], color=COLORS[r], label=r, alpha=0.85)
ax.set_xlabel(TIME_LABEL)
ax.set_ylabel('Distance from centroid (m)')
ax.set_title(f'Per-robot deviation from group centroid ({LABEL})')
ax.legend()
ax.grid(True, alpha=0.4)
fig.tight_layout()
out3 = os.path.join(OUT_DIR, f'centroid_deviation{SUFFIX}.png')
fig.savefig(out3)
plt.close(fig)
print(f"Saved: {out3}")


# ── summary stats ─────────────────────────────────────────────────────────
print("\n── Cohesion summary ──────────────────────────────────────")
print(f"  Duration           : {t[-1]:.1f} {TIME_LABEL}  ({N} samples)")
print(f"  Mean pairwise dist : {np.mean(mean_dist):.3f} m")
print(f"  Std  pairwise dist : {np.std(mean_dist):.3f} m")
print(f"  Min  pairwise dist : {np.min(mean_dist):.3f} m")
print(f"  Max  pairwise dist : {np.max(mean_dist):.3f} m")
for a, b in pairs:
    d = pair_dists[(a, b)]
    print(f"  {a}–{b}: mean={np.mean(d):.3f} m  std={np.std(d):.3f} m")
