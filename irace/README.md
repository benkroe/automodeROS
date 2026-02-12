# irace — Foraging FSM Optimisation

Automated parameter tuning for the 3-state foraging FSM using
irace.

---

## What it optimises

For each of the 3 FSM states irace selects:

| Parameter | Options |
|---|---|
| `s{i}_behavior` | 0=exploration, 1=stop, 2=phototaxis, 3=anti-phototaxis, 4=attraction, 5=repulsion |
| `c{i}_type` | 0=floor_color, 1=white_floor, 2=black_floor, 3=neighbour_count, 4=inverted_neighbours, 5=fixed_probability |
| `t{i}_offset` | 0 or 1 — which next state to jump to on condition true |

Fitness = total foraging score returned by the simulator (irace minimises, so the
target-runner returns the negative score).

---

## Prerequisites

1. **R + irace package**
   ```bash
   Rscript -e "install.packages('irace', repos='https://cloud.r-project.org')"
   ```
2. **ROS 2 workspace sourced** — the target-runner launches `ros2 launch`, so the
   environment must be set up:
   ```bash
   source /home/ben/ros2_ws/automodeROS/install/setup.bash
   ```
3. **Gazebo available** and the foraging launch file working:
   ```bash
   ros2 launch automode_tools start_foraging.launch.py \
       controller_type:=fsm fsm_config:="--fsm-config --nstates 1 --s0 0"
   ```

---

## Files

| File | Purpose |
|---|---|
| `scenario.txt` | irace scenario: budget, paths, seed |
| `parameters.txt` | Search space definition |
| `target-runner` | Bash wrapper calling `irace_foraging.py` |
| `irace_foraging.py` | Python target runner — launches simulation, returns score |
| `run_irace.R` | R script that starts irace with the scenario |
| `instances.txt` | Single dummy instance (simulation is the instance) |
| `fsm_rules.yaml` | Human-readable rules the parameter space is based on |
| `requirements.md` | Design notes |

---

## Starting irace

### Foreground (see live output)
```bash
cd /home/ben/ros2_ws/automodeROS
Rscript irace/run_irace.R
```

### Background (recommended for long runs)
```bash
cd /home/ben/ros2_ws/automodeROS
nohup Rscript irace/run_irace.R > irace/irace_output.log 2>&1 &
# Monitor progress:
tail -f irace/irace_output.log
```

---

## Tuning the budget

Edit `scenario.txt`:

```
maxExperiments = 60   # total simulator calls (60 × 4 min ≈ 4 h wall-time)
```

Edit `target-runner` to change per-run sim time:

```bash
export IRACE_SIM_MINUTES=4   # minutes of simulation per evaluation
```

Set `IRACE_VERBOSE=1` in `target-runner` for detailed per-run logging.

---

## Output

| File | Content |
|---|---|
| `irace_log.txt` | irace internal log (configurations tested, statistics) |
| `irace/runs.log` | JSON lines — one entry per simulator call with params, FSM config and score |

To inspect the best configuration after the run:

```r
library(irace)
load("irace_log.txt")   # loads object `iraceResults`
print(getFinalElites(iraceResults))
```
