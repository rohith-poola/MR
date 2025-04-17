# This code attempts to restart the simulator for every sequence in the samples.
# result - 

import numpy as np
import pybullet as p
import pybullet_data
import time
import csv
import os
from datetime import datetime

# === MPPI Settings ===
N = 5
H = 3
sigma = 0.05
lambda_ = 1.0
goal_pos = [0.4, -0.4, 0.2]
output_dir = "/Users/rohith/MS/MR/Project1/data"
os.makedirs(output_dir, exist_ok=True)
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
output_csv_path = os.path.join(output_dir, f"mppi_ee_pos_restart_sim_{timestamp}.csv")

# === Helper: One-time nominal sequence generation ===
def get_nominal_sequence(start_joint_angles, goal_pos, H):
    # Start a temporary PyBullet session
    physics_id = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
    num_joints = p.getNumJoints(robot_id)
    ee_link_index = num_joints - 1

    for j in range(num_joints):
        p.resetJointState(robot_id, j, start_joint_angles[j])

    ee_start = p.getLinkState(robot_id, ee_link_index)[0]
    waypoints = np.linspace(ee_start, goal_pos, num=H + 1)[1:]  # shape (H, 3)
    print(f"Waypoints: {waypoints}")
    joint_angle_sequence = []

    for pos in waypoints:
        ik_solution = p.calculateInverseKinematics(robot_id, ee_link_index, pos,
                                                   maxNumIterations=200, residualThreshold=1e-5)
        joint_angle_sequence.append(ik_solution[:num_joints])
    
    print(f"Joint angle sequence: {joint_angle_sequence}")
    p.disconnect()
    return np.array(joint_angle_sequence)

# === Save CSV header ===
with open(output_csv_path, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["iteration", "sequence_index", "timestep_index", "x", "y", "z"])

# === Define initial joint configuration ===
start_joint_angles = np.array([0, 0, 0, 0, 0, 0, 0])

# === Generate nominal sequence once ===
nominal_sequence = get_nominal_sequence(start_joint_angles, goal_pos, H)
noise = np.random.normal(0, sigma, size=(N, H, len(start_joint_angles)))
samples = nominal_sequence[np.newaxis, :, :] + noise  # shape (N, H, 7)
print(f'samples = {samples}')

# === For each noisy sequence, reset PyBullet and run ===
for seq_id, sequence in enumerate(samples, start=1):
    physics_id = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
    num_joints = p.getNumJoints(robot_id)
    ee_link_index = num_joints - 1

    # Reset to initial joint configuration
    for j in range(num_joints):
        p.resetJointState(robot_id, j, start_joint_angles[j])
        p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL, targetPosition=start_joint_angles[j], force=300)
    for _ in range(10):
        p.stepSimulation()

    # Execute sequence and log EE positions
    with open(output_csv_path, "a", newline="") as csvfile:
        writer = csv.writer(csvfile)

        # Log starting point (sequence_index=0) once for this iteration
        if seq_id == 1:
            ee_pos = p.getLinkState(robot_id, ee_link_index)[0]
            writer.writerow([1, 0, 0, *ee_pos])

        for t in range(H):
            joint_angles = sequence[t]
            for j in range(num_joints):
                p.setJointMotorControl2(robot_id, j, p.POSITION_CONTROL, targetPosition=joint_angles[j], force=300)
            p.stepSimulation()
            ee_pos = p.getLinkState(robot_id, ee_link_index)[0]
            writer.writerow([1, seq_id, t, *ee_pos])

    p.disconnect()
    print(f"Sequence {seq_id} complete.")

print(f"\nAll sequences done. CSV saved to:\n{output_csv_path}")
