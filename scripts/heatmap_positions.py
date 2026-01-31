#!/usr/bin/env python3

import csv
import matplotlib.pyplot as plt
import numpy as np
import os

# Path to the CSV file
csv_file_path = '/home/ben/ros2_ws/automodeROS/data/positions_log.csv'

# Check if file exists
if not os.path.exists(csv_file_path):
    print(f"Error: {csv_file_path} does not exist.")
    exit(1)

# Collect all positions
all_x = []
all_y = []

with open(csv_file_path, 'r') as f:
    reader = csv.reader(f)
    header = next(reader)
    for row in reader:
        if len(row) < 9:
            continue
        try:
            for i in range(4):  # 4 robots
                x = row[1 + 2*i]
                y = row[2 + 2*i]
                if x != 'NaN' and y != 'NaN':
                    all_x.append(float(x))
                    all_y.append(float(y))
        except ValueError:
            continue

if not all_x:
    print("No valid position data found.")
    exit(1)

# Create 2D histogram
bins = 50  # Adjust for resolution
H, xedges, yedges = np.histogram2d(all_x, all_y, bins=bins)

# Plot heat map
plt.figure(figsize=(10, 8))
plt.imshow(H.T, origin='lower', extent=[xedges[0], xedges[-1], yedges[0], yedges[-1]], cmap='hot', aspect='equal')
plt.colorbar(label='Density')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Heat Map of Robot Positions')
plt.grid(True, alpha=0.3)

# Save the plot
output_path = '/home/ben/ros2_ws/automodeROS/data/robot_heatmap.png'
plt.savefig(output_path)
print(f"Heat map saved to {output_path}")

# Optionally show
# plt.show()