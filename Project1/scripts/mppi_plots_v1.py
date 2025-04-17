import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# === Load CSV ===
csv_path = "/Users/rohith/MS/MR/Project1/data/mppi_fk_ee_2025-04-17_16-58-56.csv"  # Change this to your actual file
df = pd.read_csv(csv_path)

# === Filter iteration 1 ===
df_iter1 = df[df["iteration"] == 1]

# === Set up 3D plot ===
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# === Plot sequence 0 (initial pose) as red dot ===
seq0 = df_iter1[(df_iter1["sequence_index"] == 0) & (df_iter1["timestep_index"] == 0)]
ax.scatter(seq0["x"], seq0["y"], seq0["z"], color='red', s=80, label='Initial Pose (Seq 0)')

# === Plot sequences 1 to 25 ===
for seq_id in range(1, 26):
    seq_data = df_iter1[df_iter1["sequence_index"] == seq_id]
    ax.plot(seq_data["x"], seq_data["y"], seq_data["z"], label=f"Seq {seq_id}")

# === Labels and legend ===
ax.set_title("MPPI EE Trajectories - Iteration 1")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
plt.legend(loc='upper left', bbox_to_anchor=(1.05, 1.0), fontsize='small')
plt.tight_layout()
plt.show()
