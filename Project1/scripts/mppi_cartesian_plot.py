import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import sys

# === Path to CSV file ===
csv_path = "/Users/rohith/MS/MR/Project1/data/mppi_cartesian_samples_2025-04-21_19-51-07.csv"
# save_dir = "/Users/rohith/MS/MR/Project1/plots/mppi_cartesian_v1"
save_dir = None  # Uncomment this line to just display plots

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
    points_flat = row.iloc[2:].values.astype(float)
    sample_points = points_flat.reshape((100, 3))
    centroid = np.mean(sample_points, axis=0)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.view_init(elev=10, azim=-25)

    ax.scatter(sample_points[:, 0], sample_points[:, 1], sample_points[:, 2], color='gray', alpha=0.4, label='Sample Points')
    ax.scatter(centroid[0], centroid[1], centroid[2], color='red', s=25, label='Nominal Point')

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
