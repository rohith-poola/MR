# ========= Description & Logs =================
# Taken the optimal sequence from previous iteration as nominal sequence for the MPPI algorithm.
# Use inverse kinematics to guide the MPPI for nominal sequence.
# Result : Working!!! Corrected the MPPI formulation.
# ==============================================

import numpy as np
import pybullet as p
import pybullet_data
import csv
import os
from datetime import datetime
import time

start_time = time.time()

# === Helper Functions ===
def generate_linear_waypoints(start, end, num_points=10):
    start = np.array(start)
    end = np.array(end)
    waypoints = np.linspace(start, end, num=num_points)
    return waypoints

# Function to get the intermediate push point which is collinear to the box_center and goal position
def get_goal_points(box_pos, goal_pos, offset=0.2):
    box_pos = np.array(box_pos[:2])
    goal_pos = np.array(goal_pos[:2])

    direction = box_pos - goal_pos
    direction_unit = direction / np.linalg.norm(direction)

    push_point_xy = box_pos + direction_unit * 0.35
    return [push_point_xy[0], push_point_xy[1], 0.2]



# === MPPI Parameters ===
N = 25                # Number of sequences
H = 5                # Time horizon
sigma = 0.05          # Noise level
lambda_ = 1.0         # Temperature
threshold_dist = 0.1
max_iters = 50

# === Goal Points ===
# goal_main_points = [
#     [0.4, -0.4, 0.2],
#     [0.4, 0.4, 0.2]
# ]
box_pos = [0.4, 0, 0.2]
goal_pos = [0.4, -0.4, 0.2]
push_point = get_goal_points(box_pos, goal_pos, offset=0.4)
goal_main_points = [push_point, goal_pos]


# Generate smooth trajectory
array1 = generate_linear_waypoints(goal_main_points[0], goal_main_points[1], num_points=4)
array2 = generate_linear_waypoints(goal_main_points[1], goal_main_points[2], num_points=4)
goal_points = np.concatenate((array1, array2[1:]), axis=0)

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
    writer.writerow(["goal_index", "iteration", "sequence_index", "timestep_index", "x", "y", "z"])

# === Initial Joint Angles ===
home_joint_angles = np.zeros(num_joints)

# === MPPI Outer Loop for each goal ===
for goal_idx, goal_pos in enumerate(goal_points, start=1):
    print(f"\n=== Moving to Goal {goal_idx}: {goal_pos} ===")
    for iteration in range(1, max_iters + 1):
        # --- Step 1: Set home config and compute FK ---
        for j in range(num_joints):
            p.resetJointState(robot_id, j, home_joint_angles[j])
        home_pos = p.getLinkState(robot_id, ee_link_index)[0]

        if np.linalg.norm(np.array(home_pos) - np.array(goal_pos)) < threshold_dist:
            print(f"Goal {goal_idx} reached in {iteration - 1} iterations.")
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
        samples = nominal_sequence[np.newaxis, :, :] + noise  # (N, H, num_joints)

        # --- Step 4: Simulate all sequences and compute costs ---
        total_costs = np.zeros(N)
        all_ee_positions = []
        all_ee_positions.append((goal_idx, iteration, 1, -1, *home_pos))

        for seq_id in range(N):
            sequence = samples[seq_id]
            cumulative_cost = 0.0
            for t in range(H):
                for j in range(num_joints):
                    p.resetJointState(robot_id, j, sequence[t][j])
                ee_pos = p.getLinkState(robot_id, ee_link_index)[0]
                dist_to_goal = np.linalg.norm(np.array(ee_pos) - np.array(goal_pos))
                g_collision = 100 if ee_pos[2] < 0.1 else 0.0
                cost = dist_to_goal + g_collision
                cumulative_cost += cost 
                all_ee_positions.append((goal_idx, iteration, seq_id + 1, t, *ee_pos))
            total_costs[seq_id] = cumulative_cost

        # --- Step 5: Importance sampling ---
        weights = np.exp(-1.0 / lambda_ * (total_costs - np.min(total_costs)))
        weights /= np.sum(weights)
        optimal_sequence = np.tensordot(weights, samples, axes=(0, 0))  # (H, num_joints)

        # --- Step 6: Apply first control input for next iteration ---
        home_joint_angles = optimal_sequence[0]

        # --- Step 7: Save EE positions to CSV ---
        with open(output_csv_path, "a", newline="") as csvfile:
            writer = csv.writer(csvfile)
            for row in all_ee_positions:
                writer.writerow(row)

        print(f"[G{goal_idx}] Iter {iteration} done. EE: {np.round(home_pos, 3)} → {goal_pos}")

    else:
        print(f"Failed to reach Goal {goal_idx} in {max_iters} iterations.")

# === Cleanup ===
p.disconnect()
print(f"\nEE trajectory saved to {output_csv_path}")
print(f'===================================================')
print(f"Script took {time.time() - start_time:.2f} seconds.")
print(f'===================================================')