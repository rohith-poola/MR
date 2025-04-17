import numpy as np
import pybullet as p
import pybullet_data
import csv
import os

from datetime import datetime

# === HEADLESS SIM SETUP ===
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
num_joints = p.getNumJoints(robot_id)

goal_pos = [0.6, 0.0, 0.4]

# MPPI parameters
N = 100
H = 15
lambda_ = 1.0
sigma = 0.05

applied_actions = []

def get_joint_angles():
    return np.array([p.getJointState(robot_id, i)[0] for i in range(num_joints)])

def set_joint_angles(joint_angles):
    for i in range(num_joints):
        p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=joint_angles[i], force=200)

def get_end_effector_position():
    return np.array(p.getLinkState(robot_id, num_joints - 1)[0])

def is_self_collision():
    return len(p.getContactPoints(bodyA=robot_id, bodyB=robot_id)) > 0

def is_ground_collision():
    return len(p.getContactPoints(bodyA=robot_id, bodyB=-1)) > 0

def compute_cost(ee_pos, goal, collision_penalty):
    dist_cost = np.linalg.norm(ee_pos - goal)
    return dist_cost + collision_penalty

# === MPPI MAIN LOOP ===
for step in range(1000):
    state_id = p.saveState()
    costs = []
    all_action_seqs = []

    for _ in range(N):
        p.restoreState(state_id)
        joint_traj = []
        curr_joint_angles = get_joint_angles()
        total_cost = 0

        for _ in range(H):
            delta = np.random.normal(0, sigma, num_joints)
            new_angles = curr_joint_angles + delta
            set_joint_angles(new_angles)
            p.stepSimulation()
            curr_joint_angles = new_angles
            joint_traj.append(delta)

            ee_pos = get_end_effector_position()
            collision_penalty = 10.0 if is_self_collision() or is_ground_collision() else 0.0
            total_cost += compute_cost(ee_pos, goal_pos, collision_penalty)

        costs.append(total_cost)
        all_action_seqs.append(joint_traj)

    # Softmin importance weights
    costs = np.array(costs)
    weights = np.exp(-1.0 / lambda_ * (costs - np.min(costs)))
    weights /= np.sum(weights)

    # Optimal weighted action (first timestep only)
    avg_action = np.zeros(num_joints)
    for i in range(N):
        avg_action += weights[i] * all_action_seqs[i][0]

    # Apply optimal action to real sim
    p.restoreState(state_id)
    final_angles = get_joint_angles() + avg_action
    set_joint_angles(final_angles)
    p.stepSimulation()

    applied_actions.append(final_angles.tolist())

    p.removeState(state_id)

    # Optional stopping condition
    if np.linalg.norm(get_end_effector_position() - goal_pos) < 0.03:
        print(f"Goal reached at step {step}")
        break

p.disconnect()

# === Save MPPI applied actions to a timestamped CSV ===

save_dir = "/Users/rohith/MS/MR/Project1/data/"
os.makedirs(save_dir, exist_ok=True)
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = f"mppi_kuka_{timestamp}.csv"
filepath = os.path.join(save_dir, filename)

with open(filename, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([f"joint_{i}" for i in range(num_joints)])
    writer.writerows(applied_actions)

print(f"Saved applied actions to '{filename}'")