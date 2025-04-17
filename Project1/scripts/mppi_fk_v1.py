import numpy as np
import pybullet as p
import pybullet_data
import csv
import os
from datetime import datetime

# === MPPI + Environment Parameters ===
N = 25
H = 10
sigma = 0.05
lambda_ = 1.0
goal_pos = [0.4, 0.4, 0.2]
output_dir = "/Users/rohith/MS/MR/Project1/data"
os.makedirs(output_dir, exist_ok=True)
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
output_csv_path = os.path.join(output_dir, f"mppi_fk_ee_{timestamp}.csv")

# === Start PyBullet (DIRECT, kinematic use only) ===
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
num_joints = p.getNumJoints(robot_id)
ee_link_index = num_joints - 1

# === Use this home pose for all sequences ===
home_joint_angles = [0, 0, 0, 0, 0, 0, 0]

# === Generate nominal joint sequence using IK from home → goal ===
for j in range(num_joints):
    p.resetJointState(robot_id, j, home_joint_angles[j])
home_pos = p.getLinkState(robot_id, ee_link_index)[0]
waypoints = np.linspace(home_pos, goal_pos, num=H + 1)[1:]

nominal_sequence = []
for pos in waypoints:
    ik_solution = p.calculateInverseKinematics(robot_id, ee_link_index, pos,
                                               maxNumIterations=200, residualThreshold=1e-5)
    nominal_sequence.append(ik_solution[:num_joints])
nominal_sequence = np.array(nominal_sequence)

# === Generate noisy samples ===
noise = np.random.normal(0, sigma, size=(N, H, num_joints))
samples = nominal_sequence[np.newaxis, :, :] + noise  # (N, H, 7)

# === Save CSV header ===
with open(output_csv_path, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["iteration", "sequence_index", "timestep_index", "x", "y", "z"])

    iteration = 1
    for seq_id in range(N):
        sequence = samples[seq_id]
        for t in range(H):
            joint_angles = sequence[t]
            for j in range(num_joints):
                p.resetJointState(robot_id, j, joint_angles[j])

            ee_pos = p.getLinkState(robot_id, ee_link_index)[0]
            print(f"Sequence {seq_id + 1}/{N} | timestep {t} | EE Position: {ee_pos}")
            writer.writerow([iteration, seq_id + 1, t, *ee_pos])

print(f"\n EE positions written to: {output_csv_path}")
p.disconnect()
