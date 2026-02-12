# scripts/

Standalone post-processing and analysis scripts. These run without ROS — just Python 3 with NumPy and Matplotlib installed.

All scripts read from `data/` and write output PNGs back into the same data subdirectory.

---

## Flocking analysis

### `analyze_flocking.py`
Cohesion metrics for a single flocking experiment (FSM, BT, or Exploration baseline).

```bash
python3 scripts/analyze_flocking.py --mode fsm   # default
python3 scripts/analyze_flocking.py --mode bt
python3 scripts/analyze_flocking.py --mode exp
```

**Inputs:** `data/flocking/flocking_{mode}_log.csv`, `data/flocking/flocking_{mode}.txt` (for RTF)  
**Outputs:** `pairwise_distances[_mode].png`, `cohesion[_mode].png`, `centroid_deviation[_mode].png`

---

### `plot_rg_comparison.py`
Plots radius of gyration $R_g$ for all three experiments on one figure.

```bash
python3 scripts/plot_rg_comparison.py
```

**Inputs:** All three flocking CSV and log files  
**Output:** `data/flocking/rg_comparison.png`

---

### `plot_trajectories_all.py`
Full-run robot trajectories for FSM, BT, and Exploration — one PNG per experiment.

```bash
python3 scripts/plot_trajectories_all.py
```

**Inputs:** All three flocking CSV files  
**Outputs:** `data/flocking/trajectories_{fsm,bt,exp}.png`

---

### `plot_trajectories_flocking.py`
Trajectories for the **first 5 minutes** of the FSM experiment, showing only two selected robots.

```bash
python3 scripts/plot_trajectories_flocking.py
```

**Inputs:** `data/flocking/flocking_fsm_log.csv`  
**Output:** `data/flocking/trajectories_first5min_tb1_tb2.png`

---

## Recruitment analysis

### `analyze_recruitment.py`
Parses raw ROS log output from a recruitment experiment and computes swarm metrics: stabilisation time, join/leave rates, group size evolution, and neighbour-count variance.

```bash
python3 scripts/analyze_recruitment.py path/to/ros_log.txt
```

**Output:** `data/recruitment/recruitment_analysis.png`

---

### `plot_positions.py`
Robot trajectory plot from a recruitment `positions_log.csv`.

```bash
python3 scripts/plot_positions.py
```

**Input:** `data/recruitment/positions_log.csv`  
**Output:** `data/recruitment/robot_trajectories.png`

---

### `heatmap_positions.py`
Position heatmap from a recruitment `positions_log.csv`.

```bash
python3 scripts/heatmap_positions.py
```

**Input:** `data/recruitment/positions_log.csv`  
**Output:** `data/recruitment/robot_heatmap.png`

---

## Notes

- The irace scripts (`irace_foraging.py`, `run_irace.R`) live in `irace/`, not here.
- Scripts that log RTF from the `.txt` experiment log automatically convert wall-clock timestamps to simulation time. If no RTF is found and timestamps are already small (sim-time), no scaling is applied.
