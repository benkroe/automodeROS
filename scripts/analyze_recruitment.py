#!/usr/bin/env python3
"""
Script to analyze recruitment behavior logs for self-organized swarm metrics.

Metrics computed:
- Stabilization: Time to reach stable group sizes (variance < threshold)
- Redistribution: Join/leave event rates and density-dependent behavior
- Balancing: Variance in neighbor counts across robots over time
- Group size evolution: Average neighbor count and distribution over time
"""

import re
import matplotlib.pyplot as plt
from collections import defaultdict
import numpy as np

def parse_log_line(line):
    """Parse a single log line and extract relevant data."""
    # Match patterns for metrics
    state_pattern = r'\[Recruitment Metrics\] STATE: (\w+), n=(\d+), time=([\d.]+)'
    join_pattern = r'\[Recruitment Metrics\] JOIN: n=(\d+), time=([\d.]+)'
    leave_pattern = r'\[Recruitment Metrics\] LEAVE: n=(\d+), time=([\d.]+)'

    robot_match = re.search(r'\[(\w+)\.behavior_node\]', line)
    robot = robot_match.group(1) if robot_match else None

    # Check for state update
    state_match = re.search(state_pattern, line)
    if state_match:
        state, n, time = state_match.groups()
        return {'type': 'state', 'robot': robot, 'state': state, 'n': int(n), 'time': float(time)}

    # Check for join event
    join_match = re.search(join_pattern, line)
    if join_match:
        n, time = join_match.groups()
        return {'type': 'join', 'robot': robot, 'n': int(n), 'time': float(time)}

    # Check for leave event
    leave_match = re.search(leave_pattern, line)
    if leave_match:
        n, time = leave_match.groups()
        return {'type': 'leave', 'robot': robot, 'n': int(n), 'time': float(time)}

    return None

def analyze_recruitment_log(log_file):
    """Analyze the recruitment log file."""
    data = defaultdict(list)
    events = []

    with open(log_file, 'r') as f:
        for line in f:
            parsed = parse_log_line(line)
            if parsed:
                if parsed['type'] == 'state':
                    data[parsed['robot']].append({
                        'time': parsed['time'],
                        'n': parsed['n'],
                        'state': parsed['state']
                    })
                else:
                    events.append(parsed)

    # Sort data by time for each robot
    for robot in data:
        data[robot].sort(key=lambda x: x['time'])

    return data, events

def compute_metrics(data, events):
    """Compute the key metrics from parsed data."""
    robots = list(data.keys())
    if not robots:
        return {}

    # Get time range
    all_times = []
    for robot in robots:
        if data[robot]:
            all_times.extend([d['time'] for d in data[robot]])
    if not all_times:
        return {}
    min_time = min(all_times)
    max_time = max(all_times)
    time_range = max_time - min_time

    # Stabilization: Time when variance in n drops below threshold
    time_points = np.linspace(min_time, max_time, 50)
    variances = []
    means = []

    for t in time_points:
        n_values = []
        for robot in robots:
            # Find closest measurement to this time
            closest = min(data[robot], key=lambda x: abs(x['time'] - t), default=None)
            if closest:
                n_values.append(closest['n'])
        if n_values:
            variances.append(np.var(n_values))
            means.append(np.mean(n_values))
        else:
            variances.append(0)
            means.append(0)

    # Find stabilization time (when variance stays below 0.5 for 30 seconds)
    stabilization_time = None
    stable_threshold = 0.5
    stable_duration = 30.0

    for i, (t, var) in enumerate(zip(time_points, variances)):
        if var < stable_threshold:
            # Check if stable for next stable_duration seconds
            stable_end = t + stable_duration
            stable = True
            for j in range(i, len(time_points)):
                if time_points[j] > stable_end:
                    break
                if variances[j] >= stable_threshold:
                    stable = False
                    break
            if stable:
                stabilization_time = t - min_time
                break

    # Redistribution: Join/leave rates
    join_events = [e for e in events if e['type'] == 'join']
    leave_events = [e for e in events if e['type'] == 'leave']

    join_rate = len(join_events) / time_range * 60  # per minute
    leave_rate = len(leave_events) / time_range * 60

    # Density-dependent analysis
    join_by_density = defaultdict(int)
    leave_by_density = defaultdict(int)

    for e in join_events:
        join_by_density[e['n']] += 1
    for e in leave_events:
        leave_by_density[e['n']] += 1

    # Balancing: Average variance over time
    avg_variance = np.mean(variances) if variances else 0

    # Additional stats
    max_group_size = max(means) if means else 0
    min_group_size = min(means) if means else 0
    final_group_size = means[-1] if means else 0
    total_joins = len(join_events)
    total_leaves = len(leave_events)
    avg_join_interval = time_range / total_joins if total_joins else float('inf')
    avg_leave_interval = time_range / total_leaves if total_leaves else float('inf')
    peak_variance = max(variances) if variances else 0
    time_to_peak_variance = time_points[variances.index(peak_variance)] - min_time if variances and peak_variance > 0 else 0

    # New stats: time with group size conditions
    time_step = time_range / (len(time_points) - 1) if len(time_points) > 1 else 0
    time_less_than_2 = sum(time_step for m in means if m < 1)
    time_more_than_2 = sum(time_step for m in means if m > 1)
    time_exactly_2 = sum(time_step for m in means if 0.8 <= m <= 1.2)
    num_joins_at_n1 = sum(1 for e in join_events if e['n'] == 1)

    # Additional performance stats for pair formation at markers
    pair_formation_rate = num_joins_at_n1 / time_range * 60 if time_range > 0 else 0  # per minute
    total_time_in_pairs = time_exactly_2
    pair_success_ratio = num_joins_at_n1 / total_joins * 100 if total_joins > 0 else 0
    time_to_first_pair = min((e['time'] for e in join_events if e['n'] == 1), default=float('inf'))

    # Average pair lifetime: for each robot, sum durations of recruited periods with n=1
    pair_lifetimes = []
    for robot, robot_data in data.items():
        prev_time = None
        in_pair = False
        for d in robot_data:
            if d['state'] == 'recruited' and d['n'] == 1:
                if not in_pair:
                    prev_time = d['time']
                    in_pair = True
            elif in_pair:
                pair_lifetimes.append(d['time'] - prev_time)
                in_pair = False
        if in_pair:  # if still in pair at end
            pair_lifetimes.append(max_time - prev_time)
    avg_pair_lifetime = sum(pair_lifetimes) / len(pair_lifetimes) if pair_lifetimes else 0

    # Pair reformation rate: number of times a robot leaves and rejoins at n=1
    reformation_count = 0
    for robot, robot_data in data.items():
        left_recently = False
        for d in robot_data:
            if d['state'] == 'free' and left_recently:
                # just left, now free
                pass
            elif d['state'] == 'recruited' and d['n'] == 1 and left_recently:
                reformation_count += 1
                left_recently = False
            elif d['state'] == 'leaving':
                left_recently = True
            else:
                left_recently = False

    # Group size evolution
    avg_n_over_time = means

    return {
        'stabilization_time': stabilization_time,
        'join_rate': join_rate,
        'leave_rate': leave_rate,
        'avg_variance': avg_variance,
        'time_points': time_points,
        'variances': variances,
        'means': means,
        'join_by_density': dict(join_by_density),
        'leave_by_density': dict(leave_by_density),
        'total_time': time_range,
        'robots': len(robots),
        'max_group_size': max_group_size,
        'min_group_size': min_group_size,
        'final_group_size': final_group_size,
        'total_joins': total_joins,
        'total_leaves': total_leaves,
        'avg_join_interval': avg_join_interval,
        'avg_leave_interval': avg_leave_interval,
        'peak_variance': peak_variance,
        'time_to_peak_variance': time_to_peak_variance,
        'time_less_than_2': time_less_than_2,
        'time_more_than_2': time_more_than_2,
        'time_exactly_2': time_exactly_2,
        'num_joins_at_n1': num_joins_at_n1,
        'pair_formation_rate': pair_formation_rate,
        'total_time_in_pairs': total_time_in_pairs,
        'avg_pair_lifetime': avg_pair_lifetime,
        'pair_success_ratio': pair_success_ratio,
        'time_to_first_pair': time_to_first_pair,
        'pair_reformation_rate': reformation_count
    }

def plot_metrics(metrics, data):
    """Create plots for the metrics."""
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 8))

    # Plot 1: Variance over time (stabilization)
    ax1.plot(metrics['time_points'], metrics['variances'], 'b-')
    ax1.axhline(y=0.5, color='r', linestyle='--', label='Stability threshold')
    if metrics['stabilization_time']:
        ax1.axvline(x=metrics['stabilization_time'] + min(metrics['time_points']), color='g', linestyle='--', label=f'Stabilized at {metrics["stabilization_time"]:.1f}s')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Variance in neighbor count')
    ax1.set_title('Stabilization: Variance Over Time')
    ax1.legend()
    ax1.grid(True)

    # Plot 2: Average neighbor count over time
    ax2.plot(metrics['time_points'], metrics['means'], 'g-')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Average neighbor count')
    ax2.set_title('Group Size Evolution')
    ax2.grid(True)

    # Plot 3: Join/leave by density
    densities = sorted(set(list(metrics['join_by_density'].keys()) + list(metrics['leave_by_density'].keys())))
    joins = [metrics['join_by_density'].get(d, 0) for d in densities]
    leaves = [metrics['leave_by_density'].get(d, 0) for d in densities]

    ax3.bar([d-0.2 for d in densities], joins, width=0.4, label='Joins', alpha=0.7)
    ax3.bar([d+0.2 for d in densities], leaves, width=0.4, label='Leaves', alpha=0.7)
    ax3.set_xlabel('Neighbor count at event')
    ax3.set_ylabel('Number of events')
    ax3.set_title('Redistribution: Events by Density')
    ax3.legend()
    ax3.grid(True)

    # Plot 4: Per-robot neighbor count over time
    for robot, robot_data in data.items():
        times = [d['time'] for d in robot_data]
        ns = [d['n'] for d in robot_data]
        ax4.plot(times, ns, label=robot, marker='o', markersize=2)

    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Neighbor count')
    ax4.set_title('Per-Robot Neighbor Count')
    ax4.legend()
    ax4.grid(True)

    plt.tight_layout()
    plt.savefig('recruitment_analysis.png', dpi=150, bbox_inches='tight')
    plt.show()

def main():
    import sys
    log_file = sys.argv[1] if len(sys.argv) > 1 else 'recruitment.txt'

    print("Analyzing recruitment log...")
    data, events = analyze_recruitment_log(log_file)

    if not data:
        print("No data found in log file.")
        return

    metrics = compute_metrics(data, events)

    print("\n=== Recruitment Analysis Results ===")
    print(f"Total run time: {metrics['total_time']:.1f} seconds")
    print(f"Number of robots: {metrics['robots']}")
    stab_msg = f"Stabilization time: {metrics['stabilization_time']:.1f} seconds" if metrics['stabilization_time'] else "No stabilization detected"
    print(stab_msg)
    print(f"Join rate: {metrics['join_rate']:.2f} per minute")
    print(f"Leave rate: {metrics['leave_rate']:.2f} per minute")
    print(f"Average variance (balancing): {metrics['avg_variance']:.2f}")
    print(f"Peak variance: {metrics['peak_variance']:.2f}")
    print(f"Time to peak variance: {metrics['time_to_peak_variance']:.1f} seconds")
    print(f"Max group size: {metrics['max_group_size']:.2f}")
    print(f"Min group size: {metrics['min_group_size']:.2f}")
    print(f"Final group size: {metrics['final_group_size']:.2f}")
    print(f"Total joins: {metrics['total_joins']}")
    print(f"Total leaves: {metrics['total_leaves']}")
    join_int_msg = f"Average join interval: {metrics['avg_join_interval']:.1f} seconds" if metrics['avg_join_interval'] != float('inf') else "Average join interval: N/A (no joins)"
    print(join_int_msg)
    leave_int_msg = f"Average leave interval: {metrics['avg_leave_interval']:.1f} seconds" if metrics['avg_leave_interval'] != float('inf') else "Average leave interval: N/A (no leaves)"
    print(leave_int_msg)
    print(f"Time with group size < 2: {metrics['time_less_than_2']:.1f} seconds")
    print(f"Time with group size > 2: {metrics['time_more_than_2']:.1f} seconds")
    print(f"Time with group size ≈ 2: {metrics['time_exactly_2']:.1f} seconds")
    print(f"Number of joins forming pairs (n=1): {metrics['num_joins_at_n1']}")
    print(f"Pair formation rate at markers: {metrics['pair_formation_rate']:.2f} per minute")
    print(f"Total time spent in pairs at markers: {metrics['total_time_in_pairs']:.1f} seconds")
    print(f"Average pair lifetime at markers: {metrics['avg_pair_lifetime']:.1f} seconds")
    print(f"Pair formation success ratio: {metrics['pair_success_ratio']:.1f}%")
    print(f"Time to first pair at markers: {metrics['time_to_first_pair']:.1f} seconds" if metrics['time_to_first_pair'] != float('inf') else "Time to first pair: N/A")
    print(f"Pair reformation rate: {metrics['pair_reformation_rate']}")

    if metrics['stabilization_time']:
        print(f"Stabilization achieved at: {metrics['stabilization_time']:.1f} seconds")
    else:
        print("No stabilization detected within the run time.")

    print("\nJoin events by density:")
    for density, count in sorted(metrics['join_by_density'].items()):
        print(f"  n={density}: {count}")

    print("\nLeave events by density:")
    for density, count in sorted(metrics['leave_by_density'].items()):
        print(f"  n={density}: {count}")

    print("\n=== Performance Stats for Pair Formation at Markers ===")
    print("These stats measure how well robots form and maintain groups of two at marker positions.")
    print("- Pair formation rate: Joins per minute that create pairs (n=1).")
    print("- Total time in pairs: Time with average group size ≈2.")
    print("- Average pair lifetime: Average duration of pair states per robot.")
    print("- Pair success ratio: % of joins that form pairs.")
    print("- Time to first pair: Time to initial pair formation.")
    print("- Pair reformation rate: Number of times pairs dissolve and reform.")

    # Create plots
    try:
        plot_metrics(metrics, data)
        print("\nPlots saved to 'recruitment_analysis.png'")
    except ImportError:
        print("\nMatplotlib not available for plotting.")

if __name__ == '__main__':
    main()