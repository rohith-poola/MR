# ========= Description & Logs =================
# Read the CSV file and store the optimal end_effector position
# from the MPPI loop with timestep_index -1
# Get the corresponding joint angles and simulate!!!
# Working !!:)
# ==============================================


import pybullet as p
import pybullet_data
import pandas as pd
import numpy as np
import time

# === CSV Path ===
csv_file = "/Users/rohith/MS/MR/Project1/data/mppi_fk_ee_2025-04-17_20-19-39.csv"

# === Load and Filter CSV ===
df = pd.read_csv(csv_file)
df_filtered = df[df["timestep_index"] == -1].reset_index(drop=True)
print(f"Loaded {len(df_filtered)} -1 timestep rows from {csv_file}")

# === PyBullet Setup ===
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
num_joints = p.getNumJoints(robot_id)
ee_link_index = num_joints - 1

# === Compute Joint Configs from IK ===
joint_trajectory = []

for i, row in df_filtered.iterrows():
    target_pos = [row["x"], row["y"], row["z"]]
    ik_solution = p.calculateInverseKinematics(
        robot_id, ee_link_index, target_pos,
        maxNumIterations=200, residualThreshold=1e-4
    )
    joint_trajectory.append(ik_solution[:num_joints]) 

print(f"Computed joint angles for {len(joint_trajectory)} points.")

# === Simulate ===
for step_angles in joint_trajectory:
    for i in range(len(step_angles)):
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=step_angles[i],
            force=200
        )
    for _ in range(30):
        p.stepSimulation()
        time.sleep(1. / 1000.)

# === Keep GUI Open ===
while True:
    p.stepSimulation()
    time.sleep(1. / 240.)
