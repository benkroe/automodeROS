#!/usr/bin/env python3

import csv
import matplotlib.pyplot as plt
import os

# Path to the CSV file
csv_file_path = '/home/ben/ros2_ws/automodeROS/data/foraging_fsm_01.csv'

# Check if file exists
if not os.path.exists(csv_file_path):
    print(f"Error: {csv_file_path} does not exist.")
    exit(1)

# Read the CSV
timestamps = []
positions = {'tb1': [], 'tb2': [], 'tb3': [], 'tb4': []}

with open(csv_file_path, 'r') as f:
    reader = csv.reader(f)
    header = next(reader)
    # header: ['timestamp', 'tb1_x', 'tb1_y', 'tb2_x', 'tb2_y', 'tb3_x', 'tb3_y', 'tb4_x', 'tb4_y']
    for row in reader:
        if len(row) < 9:
            continue
        try:
            ts = float(row[0])
            timestamps.append(ts)
            for i, robot in enumerate(['tb1', 'tb2', 'tb3', 'tb4']):
                x = row[1 + 2*i]
                y = row[2 + 2*i]
                if x != 'NaN' and y != 'NaN':
                    positions[robot].append((float(x), float(y)))
                else:
                    positions[robot].append(None)  # or skip
        except ValueError:
            continue

# Now, plot trajectories
plt.figure(figsize=(10, 8))
colors = ['blue', 'red', 'green', 'orange']
for robot, color in zip(['tb1', 'tb2', 'tb3', 'tb4'], colors):
    x_vals = [p[0] for p in positions[robot] if p is not None]
    y_vals = [p[1] for p in positions[robot] if p is not None]
    if x_vals:
        plt.plot(x_vals, y_vals, color=color, marker='o', markersize=2, label=f'{robot} trajectory')
        # Mark start and end
        plt.scatter(x_vals[0], y_vals[0], color=color, marker='^', s=100, label=f'{robot} start')
        plt.scatter(x_vals[-1], y_vals[-1], color=color, marker='v', s=100, label=f'{robot} end')

plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Robot Trajectories from Position Log')
plt.legend()
plt.grid(True)
plt.axis('equal')  # Equal aspect ratio

# Save the plot
output_path = '/home/ben/ros2_ws/automodeROS/data/robot_trajectories.png'
plt.savefig(output_path)
print(f"Plot saved to {output_path}")

# Optionally show
# plt.show()