#!/usr/bin/env python3

import subprocess
import sys
import os
import time
import json
import yaml
import signal
import subprocess

FSM_STATE_COUNT = 3
FSM_CONDITIONS_PER_STATE = 1


def _parse_irace_params(argv):
    """Parse irace parameters from argv into a dict.

    Supports two formats:
    1) Named args: key=value or --key value
    2) irace positional: <config_id> <instance_id> <seed> <instance> <p1> ... <pN>
    """
    params = {}

    # Positional irace format detection
    if len(argv) >= 13 and all('=' not in a for a in argv[4:13]):
        ordered = [
            's0_behavior', 's1_behavior', 's2_behavior',
            'c0_type', 'c1_type', 'c2_type',
            't0_offset', 't1_offset', 't2_offset'
        ]
        values = argv[4:13]
        for key, value in zip(ordered, values):
            params[key] = value
        return params

    # Named args format
    i = 0
    while i < len(argv):
        arg = argv[i]
        if '=' in arg:
            key, value = arg.split('=', 1)
            params[key.lstrip('-')] = value
        elif arg.startswith('--') and i + 1 < len(argv):
            key = arg[2:]
            value = argv[i + 1]
            params[key] = value
            i += 1
        i += 1
    return params


def _build_fsm_config(params):
    """Build FSM config string from irace parameters."""
    s0 = int(params['s0_behavior'])
    s1 = int(params['s1_behavior'])
    s2 = int(params['s2_behavior'])

    c0 = int(params['c0_type'])
    c1 = int(params['c1_type'])
    c2 = int(params['c2_type'])

    t0 = int(params['t0_offset'])
    t1 = int(params['t1_offset'])
    t2 = int(params['t2_offset'])

    if any(t not in (0, 1) for t in (t0, t1, t2)):
        raise ValueError('target offsets must be 0 or 1 for 3-state FSM')

    return (
        f"--fsm-config --nstates {FSM_STATE_COUNT} "
        f"--s0 {s0} --n0 {FSM_CONDITIONS_PER_STATE} --n0x0 {t0} --c0x0 {c0} "
        f"--s1 {s1} --n1 {FSM_CONDITIONS_PER_STATE} --n1x0 {t1} --c1x0 {c1} "
        f"--s2 {s2} --n2 {FSM_CONDITIONS_PER_STATE} --n2x0 {t2} --c2x0 {c2} "
    )


def _append_run_log(entry):
    log_path = os.environ.get('IRACE_RUN_LOG', '/home/ben/ros2_ws/automodeROS/irace/runs.log')
    try:
        with open(log_path, 'a') as fh:
            fh.write(json.dumps(entry) + "\n")
    except Exception:
        pass


def _pgrep_user(pattern: str):
    try:
        uid = str(os.getuid())
        out = subprocess.check_output(['pgrep', '-u', uid, '-f', pattern], text=True)
        return [int(x) for x in out.strip().split() if x.strip()]
    except Exception:
        return []


def _cleanup_leftovers():
    # Kill any leftover launch or gazebo processes for this user
    subprocess.run(['pkill', '-u', str(os.getuid()), '-f', 'ros2 launch automode_tools start_foraging.launch.py'], check=False)
    subprocess.run(['pkill', '-u', str(os.getuid()), '-f', 'gzserver|gzclient|gz sim'], check=False)

    # Wait briefly for gazebo processes to exit
    for _ in range(10):
        if not _pgrep_user('gzserver|gzclient|gz sim'):
            break
        time.sleep(1)

def run_foraging_experiment(fsm_config, module_package, sim_time_minutes=5):
    """
    Run a foraging experiment with given fsm_config.
    Returns the total_score.
    """
    verbose = os.environ.get('IRACE_VERBOSE', '0') == '1'
    if verbose:
        print(f"Starting foraging experiment with fsm_config: {fsm_config[:80]}..., sim_time: {sim_time_minutes} minutes")
    
    # Modify timer for sim_time_minutes
    timer_file = '/home/ben/ros2_ws/automodeROS/src/automode_tools/automode_tools/timer_shutdown.py'
    if verbose:
        print(f"Modifying timer file: {timer_file} to {sim_time_minutes * 60} seconds")
    with open(timer_file, 'r') as f:
        content = f.read()
    # Replace the 300 with sim_time_minutes * 60
    content = content.replace('300', str(sim_time_minutes * 60))
    with open(timer_file, 'w') as f:
        f.write(content)
    
    # Launch with fsm_config arg
    cmd = [
        'ros2', 'launch', 'automode_tools', 'start_foraging.launch.py',
        f'controller_type:=fsm',
        f'module_package:={module_package}',
        f'fsm_config:={fsm_config}'
    ]
    if verbose:
        print(f"Launching command: {' '.join(cmd)}")
    proc = subprocess.Popen(
        cmd,
        cwd='/home/ben/ros2_ws/automodeROS',
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid
    )
    
    # Wait for completion or timeout
    if verbose:
        print("Waiting for simulation to complete...")
    output_lines = []
    sim_reached = False
    saw_scores = False
    start_wall = time.time()
    max_wall = sim_time_minutes * 60 * 4  # generous wall-time timeout

    while True:
        if proc.stdout is None:
            break
        line = proc.stdout.readline()
        if line:
            output_lines.append(line)
            if 'Simulation time reached' in line:
                sim_reached = True
                break
            if 'SCORES:' in line:
                saw_scores = True
        if proc.poll() is not None:
            break
        if time.time() - start_wall > max_wall:
            if verbose:
                print("Wall-time timeout reached, terminating simulation...")
            break

    # Stop the launch if still running
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except Exception:
            pass
        time.sleep(5)
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass

    _cleanup_leftovers()

    # Read remaining output
    if proc.stdout is not None:
        remaining = proc.stdout.read()
        if remaining:
            output_lines.append(remaining)

    output = ''.join(output_lines)
    if verbose:
        print("Simulation finished.")
    
    # Clean up processes (already terminated launch process group)
    if verbose:
        print("Cleanup complete.")
    
    # Parse output for RTF and scores
    rtf = None
    scores = {}
    if verbose:
        print("Parsing output for metrics...")
    for line in output.split('\n'):
        if 'RTF:' in line:
            rtf_str = line.split('RTF:')[1].split()[0]
            rtf = float(rtf_str)
            if verbose:
                print(f"Found RTF: {rtf}")
        if 'SCORES:' in line:
            scores_str = line.split('SCORES:')[1].strip()
            scores = eval(scores_str)
            if verbose:
                print(f"Found SCORES: {scores}")
    
    # Reset timer
    if verbose:
        print("Resetting timer file")
    content = content.replace(str(sim_time_minutes * 60), '300')
    with open(timer_file, 'w') as f:
        f.write(content)
    
    total_score = sum(scores.values()) if scores else 0
    if proc.returncode not in (0, None):
        return 0
    if not sim_reached or not saw_scores:
        return 0
    if verbose:
        print(f"Total score: {total_score}")
    return total_score

if __name__ == '__main__':
    params = _parse_irace_params(sys.argv[1:])
    required = ['s0_behavior', 's1_behavior', 's2_behavior', 'c0_type', 'c1_type', 'c2_type', 't0_offset', 't1_offset', 't2_offset']
    missing = [p for p in required if p not in params]
    if missing:
        print(f"Usage: python irace_foraging.py {' '.join(required)}")
        print(f"Missing params: {missing}")
        sys.exit(1)

    # Load module_package from YAML to keep it fixed for all runs
    config_file = '/home/ben/ros2_ws/automodeROS/src/automode_controller/launch/config_foraging.yaml'
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f) or {}
    module_package = config.get('module_package', 'basic_modules')

    fsm_config = _build_fsm_config(params)
    sim_minutes_env = os.environ.get('IRACE_SIM_MINUTES')
    sim_minutes = int(sim_minutes_env) if sim_minutes_env else 5
    total_score = run_foraging_experiment(fsm_config, module_package, sim_time_minutes=sim_minutes)

    _append_run_log({
        'timestamp': time.time(),
        'params': params,
        'fsm_config': fsm_config,
        'module_package': module_package,
        'sim_minutes': sim_minutes,
        'total_score': total_score
    })

    # irace minimizes by default, so return negative score as cost
    print(-total_score)