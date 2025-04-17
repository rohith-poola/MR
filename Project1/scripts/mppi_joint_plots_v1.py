import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# === Load the joint sequence CSV ===
csv_path = "/Users/rohith/MS/MR/Project1/data/mppi_joint_sequences_2025-04-17_15-19-17.csv"  # 🔁 Replace this path
df = pd.read_csv(csv_path)

# === Initial joint angles to prepend (7 DOF) ===
initial_joint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# === Filter for iteration 1 and noisy sequences only ===
df = df[(df["iteration"] == 1) & (df["sequence_index"] > 0)]

# === Constants ===
num_joints = 7
H = df["timestep_index"].max() + 1
unique_seqs = sorted(df["sequence_index"].unique())

# === Plot setup ===
fig, axes = plt.subplots(num_joints, 1, figsize=(10, 14), sharex=True)

# === Preprocess and plot each joint ===
for joint_id in range(num_joints):
    ax = axes[joint_id]
    for seq_id in unique_seqs:
        seq_data = df[df["sequence_index"] == seq_id].sort_values("timestep_index")
        joint_traj = seq_data[f"joint_{joint_id}"].values
        full_traj = np.insert(joint_traj, 0, initial_joint[joint_id])  # prepend t=0
        ax.plot(range(H + 1), full_traj, alpha=0.8)

    ax.set_ylabel(f"Joint {joint_id}")
    ax.grid(True)

axes[-1].set_xlabel("Timestep")
fig.suptitle("Joint Angle Trajectories (Prepended Initial Point)", fontsize=14)
plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.show()
