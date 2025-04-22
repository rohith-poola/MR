# ========= Description & Logs =================
# Rather that sampling the joint space let's sample the cartesian space 
# and then use inverse kinematics to get the joint angles.
# It's Working ...... Need to tune some parameters but went close to 10cm to the goal position
# =======================================

import numpy as np
import pybullet as p
import pybullet_data
import csv
import os
from datetime import datetime
import time

# === Settings ===
N = 100
step_size = 0.15
radius = 0.15
threshold_dist = 0.1
max_iters = 100

# === Sampling Helper ===
def sample_points_around(center, radius, N):
    points = []
    while len(points) < N:
        point = center + np.random.uniform(-radius, radius, size=3)
        if np.linalg.norm(point - center) <= radius:
            points.append(point)
    return np.array(points)

def direction_point(start, goal, distance):
    direction = np.array(goal) - np.array(start)
    direction /= np.linalg.norm(direction)
    return start + direction * distance

# === Logging Setup ===
output_dir = "/Users/rohith/MS/MR/Project1/data"
os.makedirs(output_dir, exist_ok=True)
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
output_csv_path = os.path.join(output_dir, f"mppi_cartesian_samples_{timestamp}.csv")

# === PyBullet Setup ===
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
num_joints = p.getNumJoints(robot_id)
ee_link_index = num_joints - 1

# === Goal Points ===
goal_points = np.array([
    [0.4, -0.4, 0.2],
    [0.4,  0.4, 0.2]
])

# === CSV Header ===
with open(output_csv_path, "w", newline="") as f:
    writer = csv.writer(f)
    header = ["goal_id", "iteration_id"]
    for i in range(N):
        header += [f"x{i}", f"y{i}", f"z{i}"]
    writer.writerow(header)

# === Initial Config ===
home_joint_angles = np.zeros(num_joints)

# === Main Loop Over Two Goals ===
for goal_id, goal_pos in enumerate(goal_points, start=1):
    print(f"\n=== Moving to Goal {goal_id}: {goal_pos} ===")

    for iteration in range(1, max_iters + 1):
        for j in range(num_joints):
            p.resetJointState(robot_id, j, home_joint_angles[j])
        home_pos = np.array(p.getLinkState(robot_id, ee_link_index)[0])

        if np.linalg.norm(home_pos - goal_pos) < threshold_dist:
            print(f"Goal {goal_id} reached in {iteration - 1} iterations.")
            break

        # --- Focus Point ---
        focus_point = direction_point(home_pos, goal_pos, step_size)
        sampled_points = sample_points_around(focus_point, radius, N)

        # --- IK + Collision + Cost ---
        costs = []
        valid_points = []
        valid_joint_configs = []

        for point in sampled_points:
            ik = p.calculateInverseKinematics(robot_id, ee_link_index, point,
                                              maxNumIterations=200, residualThreshold=1e-4)[:num_joints]
            for j in range(num_joints):
                p.resetJointState(robot_id, j, ik[j])
            ee_z = p.getLinkState(robot_id, ee_link_index)[0][2]
            if ee_z < 0.1:
                cost = 1000.0
            else:
                cost = np.linalg.norm(np.array(point) - np.array(goal_pos))
                valid_points.append(point)
                valid_joint_configs.append(ik)
            costs.append(cost)

        if len(valid_joint_configs) == 0:
            print(f"[G{goal_id}] Iter {iteration}: No valid IK solutions. Retrying...")
            continue

        # --- Importance Sampling ---
        costs = np.array(costs[:len(valid_joint_configs)])
        weights = np.exp(-1.0 / 1.0 * (costs - np.min(costs)))
        weights /= np.sum(weights)
        next_joint_angles = np.tensordot(weights, np.array(valid_joint_configs), axes=(0, 0))

        # --- Apply Action ---
        home_joint_angles = next_joint_angles

        # --- Log Samples ---
        row = [goal_id, iteration]
        row += sampled_points.flatten().tolist()
        with open(output_csv_path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(row)

        print(f"[G{goal_id}] Iter {iteration}: EE {np.round(home_pos, 3)} → {np.round(goal_pos, 3)}")

    else:
        print(f"Failed to reach Goal {goal_id} in {max_iters} iterations.")

# === Cleanup ===
p.disconnect()
print(f"\nSampled point clouds saved to {output_csv_path}")
