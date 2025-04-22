# ======= Description ========
# Visulaize the MPPI rollouts and the optimal point in 3D space.
# ============================

import pandas as pd
import matplotlib.pyplot as plt
import os
import sys

# === Path to CSV file ===
csv_path = "/Users/rohith/MS/MR/Project1/data/mppi_fk_ee_2025-04-17_20-19-39.csv"
# save_dir = None
save_dir = "/Users/rohith/MS/MR/Project1/plots/main_mppi_v4"

# === Fixed axis limits ===
xlim = [-0.8, 0.8]
ylim = [-0.8, 0.8]
zlim = [0.0, 1.6]

# === Load CSV ===
df = pd.read_csv(csv_path)
goal_indices = sorted(df["goal_index"].unique())
print(f"Loaded data for {len(goal_indices)} goals from {csv_path}")

if save_dir:
    if os.path.exists(save_dir):
        print(f"Directory '{save_dir}' already exists. Exiting.")
        sys.exit(1)
    else:
        os.makedirs(save_dir)

# === Plot each (goal, iteration) ===
for goal_idx in goal_indices:
    df_goal = df[df["goal_index"] == goal_idx]
    iterations = sorted(df_goal["iteration"].unique())

    for iteration in iterations:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.view_init(elev=10, azim=-25)

        # Filter data for this (goal, iteration)
        df_iter = df_goal[df_goal["iteration"] == iteration]
        home_row = df_iter[(df_iter["sequence_index"] == 1) & (df_iter["timestep_index"] == -1)].iloc[0]
        red_x, red_y, red_z = home_row["x"], home_row["y"], home_row["z"]
        ax.scatter(red_x, red_y, red_z, color='red', s=50, label='Start (-1)')

        for seq_id in sorted(df_iter["sequence_index"].unique()):
            df_seq = df_iter[(df_iter["sequence_index"] == seq_id) & (df_iter["timestep_index"] >= 0)]
            df_seq_sorted = df_seq.sort_values("timestep_index")

            x_vals = [red_x] + df_seq_sorted["x"].tolist()
            y_vals = [red_y] + df_seq_sorted["y"].tolist()
            z_vals = [red_z] + df_seq_sorted["z"].tolist()
            ax.plot(x_vals, y_vals, z_vals, color='blue')

        # Set fixed axis limits
        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        ax.set_zlim(zlim)

        ax.set_title(f"Goal {goal_idx} — Iteration {iteration}")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.legend()

        if save_dir:
            filename = f"goal_{goal_idx}_iter_{iteration:03d}.png"
            plt.savefig(os.path.join(save_dir, filename))
            plt.close()
        else:
            plt.show()
