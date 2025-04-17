import numpy as np
import pybullet as p
import pybullet_data
import csv
import os
from datetime import datetime
import time

start_time = time.time()

# === MPPI Parameters ===
N = 25                # Number of sequences
H = 10                # Time horizon
sigma = 0.05          # Noise level
lambda_ = 1.0         # Temperature
goal_pos = [0.4, -0.4, 0.2]
threshold_dist = 0.05
max_iters = 50

# === Output Logging Setup ===
output_dir = "/Users/rohith/MS/MR/Project1/data"
os.makedirs(output_dir, exist_ok=True)
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
output_csv_path = os.path.join(output_dir, f"mppi_fk_ee_{timestamp}.csv")

# === PyBullet DIRECT Mode ===
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
num_joints = p.getNumJoints(robot_id)
ee_link_index = num_joints - 1

# === Logging CSV Header ===
with open(output_csv_path, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["iteration", "sequence_index", "timestep_index", "x", "y", "z"])

# === MPPI Loop ===
home_joint_angles = np.zeros(num_joints)

for iteration in range(1, max_iters + 1):
    # --- Step 1: Set home config and compute FK ---
    for j in range(num_joints):
        p.resetJointState(robot_id, j, home_joint_angles[j])
    home_pos = p.getLinkState(robot_id, ee_link_index)[0]

    if np.linalg.norm(np.array(home_pos) - np.array(goal_pos)) < threshold_dist:
        print(f"Goal reached in {iteration - 1} iterations.")
        break

    # --- Step 2: Generate nominal sequence via IK ---
    waypoints = np.linspace(home_pos, goal_pos, num=H + 1)[1:]
    nominal_sequence = []
    for pos in waypoints:
        ik_solution = p.calculateInverseKinematics(robot_id, ee_link_index, pos,
                                                   maxNumIterations=200, residualThreshold=1e-5)
        nominal_sequence.append(ik_solution[:num_joints])
    nominal_sequence = np.array(nominal_sequence)

    # --- Step 3: Generate noisy samples ---
    noise = np.random.normal(0, sigma, size=(N, H, num_joints))
    samples = nominal_sequence[np.newaxis, :, :] + noise  # (N, H, 7)

    # --- Step 4: Simulate all sequences and compute costs ---
    total_costs = np.zeros(N)
    all_ee_positions = []
    all_ee_positions.append((iteration, 1, -1, *home_pos))

    for seq_id in range(N):
        sequence = samples[seq_id]
        cumulative_cost = 0.0
        for t in range(H):
            for j in range(num_joints):
                p.resetJointState(robot_id, j, sequence[t][j])
            ee_pos = p.getLinkState(robot_id, ee_link_index)[0]
            dist_to_goal = np.linalg.norm(np.array(ee_pos) - np.array(goal_pos))
            collision_penalty = 0.0  # Extend here if collision checking is needed
            cost = dist_to_goal + collision_penalty
            cumulative_cost += cost

            # Store EE position for logging
            all_ee_positions.append((iteration, seq_id + 1, t, *ee_pos))
        total_costs[seq_id] = cumulative_cost

    # --- Step 5: Importance sampling to get optimal sequence ---
    weights = np.exp(-1.0 / lambda_ * (total_costs - np.min(total_costs)))
    weights /= np.sum(weights)
    optimal_sequence = np.tensordot(weights, samples, axes=(0, 0))  # (H, num_joints)

    # --- Step 6: Apply first control input ---
    home_joint_angles = optimal_sequence[0]

    # --- Step 7: Save EE positions to CSV ---
    with open(output_csv_path, "a", newline="") as csvfile:
        writer = csv.writer(csvfile)
        for row in all_ee_positions:
            writer.writerow(row)

    print(f"Iteration {iteration} complete. EE pos: {np.round(home_pos, 3)} → Goal: {goal_pos}")

p.disconnect()
print(f"\nEE trajectory saved to {output_csv_path}")

end_time = time.time()
elapsed_time = end_time - start_time
print(f"Script took {elapsed_time:.4f} seconds.")