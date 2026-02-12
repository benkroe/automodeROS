# irace — Foraging FSM Optimisation

Automated parameter search for the 3-state foraging FSM using [irace](https://cran.r-project.org/package=irace). Each irace evaluation launches a full foraging simulation (`start_foraging.launch.py`), runs it for a fixed duration, and returns the negative foraging score (irace minimises, so lower = better configuration).

---

## Search space

A fixed 3-state FSM with **1 condition per state** is assumed. Each state contributes 3 parameters (behavior, condition type, target offset), giving **9 parameters total**:

| Parameter | Type | Choices |
|---|---|---|
| `s{0,1,2}_behavior` | categorical | 0=exploration, 1=stop, 2=phototaxis, 3=anti-phototaxis, 4=attraction, 5=repulsion |
| `c{0,1,2}_type` | categorical | 0=floor_color, 1=white_floor, 2=black_floor, 3=neighbour_count, 4=inverted_neighbours_count, 5=fixed_probability |
| `t{0,1,2}_offset` | categorical | 0 or 1 — next-state index when the condition fires |

Module parameters are not tuned; runtime defaults are used. The type is the type of the condition. 

---

## Files

| File | Purpose |
|---|---|
| `scenario.txt` | irace scenario: paths, budget, seed, parallelism |
| `parameters.txt` | Search space definition (9 categorical parameters) |
| `target-runner` | Bash wrapper called by irace per evaluation; sets `IRACE_SIM_MINUTES` and calls `irace_foraging.py` |
| `irace_foraging.py` | Builds the FSM config string from irace parameters, launches the simulation, waits, returns the score |
| `run_irace.R` | R entry point — reads the scenario and calls `irace()` |
| `instances.txt` | Single dummy instance (required by irace; the simulation itself is the instance) |
| `fsm_rules.yaml` | Human-readable description of the allowed behavior/condition types and FSM constraints |
| `requirements.md` | Design notes |

---

## Hardcoded paths

`scenario.txt`, `run_irace.R`, `target-runner`, and `irace_foraging.py` all contain absolute paths pointing to `/home/ben/ros2_ws/automodeROS`. **Before running on a different machine, update these paths** to match your workspace location:

```bash
grep -rn "/home/ben" /home/YOUR_USER/ros2_ws/automodeROS/irace/
```

---

## Running irace

### 1. Install R and the irace package

```bash
sudo apt install r-base
Rscript -e "install.packages('irace', repos='https://cloud.r-project.org')"
```

### 2. Source the ROS 2 workspace

The target-runner calls `ros2 launch`, so the workspace must be sourced in the same shell:

```bash
source ~/ros2_ws/automodeROS/install/setup.bash
```

### 3. Verify the simulation works

```bash
ros2 launch automode_tools start_foraging.launch.py
```

The launch file must complete without errors before irace is started.

### 4. Start irace

```bash
cd ~/ros2_ws/automodeROS
Rscript irace/run_irace.R
```

For long runs (recommended):

```bash
nohup Rscript irace/run_irace.R > irace/irace_output.log 2>&1 &
tail -f irace/irace_output.log
```

---

## Tuning the budget

Edit `scenario.txt` to change the number of evaluations:

```
maxExperiments = 60   # 60 evaluations × ~4 min each ≈ 4 h wall-time
```

Edit `target-runner` to change simulation duration per evaluation:

```bash
export IRACE_SIM_MINUTES=4
```

Set `IRACE_VERBOSE=1` in `target-runner` for detailed per-run logging.

---

## Output

| File | Content |
|---|---|
| `irace_log.txt` | irace internal log (created in the working directory) |
| `irace/runs.log` | JSON lines — one entry per evaluation with FSM config string and score |

Inspect the best configurations found:

```r
library(irace)
load("irace_log.txt")
print(getFinalElites(iraceResults))
```

