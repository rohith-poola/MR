# ======= Description ========
# Visulaize the sampled points and the optimal point in 3D space.
# ============================

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import sys

# === Path to CSV file ===
csv_path = "/Users/rohith/MS/MR/Project1/data/mppi_cartesian_samples_with_opt_2025-04-21_20-19-09.csv"
save_dir = "/Users/rohith/MS/MR/Project1/plots/mppi_cartesian_v1"
# save_dir = None  # Set to None to just display plots

# === Fixed axis limits ===
xlim = [-0.8, 0.8]
ylim = [-0.8, 0.8]
zlim = [0.0, 1.6]

# === Load CSV ===
df = pd.read_csv(csv_path)
print(f"Loaded {len(df)} samples from {csv_path}")

if save_dir:
    if os.path.exists(save_dir):
        print(f"Directory '{save_dir}' already exists. Exiting.")
        sys.exit(1)
    else:
        os.makedirs(save_dir)

# === Process & Plot ===
for idx, row in df.iterrows():
    goal_id = int(row["goal_id"])
    iteration_id = int(row["iteration_id"])
    
    # Parse optimal point and sampled points
    points_flat = row.iloc[2:].values.astype(float)  # (3 + 100*3 = 303)
    assert len(points_flat) == 303, f"Expected 303 values but got {len(points_flat)}"
    
    sample_points = points_flat.reshape((101, 3))  # First row is optimal point

    optimal_point = sample_points[0]
    sampled_points = sample_points[1:]
    centroid = np.mean(sampled_points, axis=0)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.view_init(elev=10, azim=-25)

    # Plot
    ax.scatter(sampled_points[:, 0], sampled_points[:, 1], sampled_points[:, 2],
               color='gray', alpha=0.4, label='Sampled Points')
    ax.scatter(optimal_point[0], optimal_point[1], optimal_point[2],
               color='green', s=25, label='Optimal Point')
    ax.scatter(centroid[0], centroid[1], centroid[2],
               color='red', s=25, label='Nominal Point')

    # Labels & aesthetics
    ax.set_title(f"Goal {goal_id} — Iteration {iteration_id}")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_zlim(zlim)
    ax.set_box_aspect([1, 1, 1])
    ax.legend()

    if save_dir:
        filename = f"goal_{goal_id}_iter_{iteration_id:03d}.png"
        plt.savefig(os.path.join(save_dir, filename))
        plt.close()
    else:
        plt.tight_layout()
        plt.show()
