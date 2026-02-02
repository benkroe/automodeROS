# irace Foraging Requirements (FSM only)

## irace_foraging.py
- **Inputs**: parameters for FSM module selection (state behaviors, condition types, target offsets).
- **Outputs**: prints a single numeric fitness (total_score) to stdout for irace.
- **FSM structure**:
  - Fixed 3 states (0,1,2).
  - Exactly 1 condition per state.
  - No module parameters in config string (defaults only).
- **Launch**:
  - `controller_type:=fsm`
  - `fsm_config:=<generated>`
  - `module_package` read from config_foraging.yaml (fixed).
- **Shutdown**: must terminate simulation and cleanup ROS/Gazebo processes per run.
- **Logging**: report config string, score, and RTF in stdout.

## scenario.txt
- **targetRunner**: `python3 scripts/irace_foraging.py`
- **parameterFile**: `irace/parameters.txt`
- **Budget**: set `maxExperiments`, `firstTest`, and `mu` for testing vs. full runs.
- **Determinism**: `deterministic = 0` (simulation is stochastic).
- **Output**: `logFile` and `tuningArchive` set for tracking.

## parameters.txt
- Defines FSM module choices only (no module parameters).
- Parameters:
  - `s0_behavior`, `s1_behavior`, `s2_behavior` (behavior type IDs)
  - `c0_type`, `c1_type`, `c2_type` (condition type IDs)
  - `t0_offset`, `t1_offset`, `t2_offset` (target offsets)

## Rules (fsm_rules.yaml)
- Explicit list of allowed behavior/condition types and constraints.
- Used to keep irace within valid FSM grammar.
