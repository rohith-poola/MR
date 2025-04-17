# ========= Description & Logs =================
# Use inverse kinematics to kick start the MPPI algorithm.
# with GUI but storing all the end effector position (N, H, 3) in a CSV file.
# 
# =======================================


import numpy as np
import pybullet as p
import pybullet_data
import time
import csv
import os
from datetime import datetime

# === Setup: PyBullet in DIRECT mode ===
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
num_joints = p.getNumJoints(robot_id)
ee_link_index = num_joints - 1
goal_pos = [0.3, 0.3, 0.3]

# === Visual goal marker ===
goal_marker_vis_id = p.createVisualShape(
    shapeType=p.GEOM_SPHERE,
    radius=0.03,
    rgbaColor=[0, 1, 0, 1]  # Green
)
p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=-1,
    baseVisualShapeIndex=goal_marker_vis_id,
    basePosition=goal_pos
)

# === MPPI Parameters ===
N = 25            # Number of sampled sequences
H = 10            # Time horizon
sigma = 0.05      # Exploration noise std
lambda_ = 1.0     # Importance sampling temperature

output_dir = "/Users/rohith/MS/MR/Project1/data"
os.makedirs(output_dir, exist_ok=True)
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
output_csv_path = os.path.join(output_dir, f"mppi_ee_pos_{timestamp}.csv")

# === MPPI Helper Functions ===
def get_joint_angles():
    return np.array([p.getJointState(robot_id, i)[0] for i in range(num_joints)])

def set_joint_angles(joint_angles):
    for i in range(num_joints):
        p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=joint_angles[i], force=300)

def get_ee_position():
    return np.array(p.getLinkState(robot_id, ee_link_index)[0])

def is_self_collision():
    contact_points = p.getContactPoints(bodyA=robot_id, bodyB=robot_id)
    if contact_points is None:
        return 0
    return len(contact_points) > 0

def is_ground_collision():
    contact_points = p.getContactPoints(bodyA=robot_id, bodyB=-1)
    if contact_points is None:
        return 0
    return len(contact_points) > 0

def compute_cost(ee_pos, goal, collision_penalty):
    return np.linalg.norm(ee_pos - goal) + collision_penalty

def get_nominal_sequence(start_pos, goal_pos, H):
    waypoints = np.linspace(start_pos, goal_pos, num=H + 1)[1:]  # shape (H, 3)
    joint_angle_sequence = []

    for pos in waypoints:
        ik_solution = p.calculateInverseKinematics(
            bodyUniqueId=robot_id,
            endEffectorLinkIndex=ee_link_index,
            targetPosition=pos,
            maxNumIterations=200,
            residualThreshold=1e-5
        )
        joint_angle_sequence.append(ik_solution[:num_joints])

    return np.array(joint_angle_sequence)

# === Initialize MPPI Loop ===
home = get_ee_position()
nominal_sequence = get_nominal_sequence(start_pos=home, goal_pos=goal_pos, H=H)
dist = np.linalg.norm(get_ee_position() - goal_pos)
iteration = 1

with open(output_csv_path, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["iteration", "sequence_index", "timestep_index", "x", "y", "z"])

    while dist > 0.05:
        state_id = p.saveState()

        # === Log current EE pos as sequence 0, timestep 0 ===
        ee_pos = get_ee_position()
        writer.writerow([iteration, 0, 0, *ee_pos])

        # === Generate samples for sequences 1 to N ===
        noise = np.random.normal(0, sigma, size=(N, H, num_joints))
        samples = nominal_sequence[np.newaxis, :, :] + noise
        total_costs = np.zeros(N)

        for i, sequence in enumerate(samples, start=1):  # sequence index starts at 1
            p.restoreState(state_id)
            p.stepSimulation()
            for _ in range(int(2.0 * 50)):  # Reset sim state warm-up
                p.stepSimulation()
                time.sleep(1. / 1000.)

            for t in range(H):
                joint_angles = sequence[t]
                set_joint_angles(joint_angles)
                p.stepSimulation()
                time.sleep(1. / 240.)

                ee_pos = get_ee_position()
                collision_penalty = 100.0 if is_self_collision() or is_ground_collision() else 0.0
                total_costs[i - 1] += compute_cost(ee_pos, goal_pos, collision_penalty)
                writer.writerow([iteration, i, t, *ee_pos])
            print(f'Iteration {iteration} | Sequence {i + 1}/{N} |')
         
        # Importance sampling
        weights = np.exp(-1.0 / lambda_ * (total_costs - np.min(total_costs)))
        weights /= np.sum(weights)
        nominal_sequence = np.tensordot(weights, samples, axes=(0, 0))

        # Apply first action of updated nominal sequence
        new_joint_angles = nominal_sequence[0]
        set_joint_angles(new_joint_angles)
        p.stepSimulation()
        time.sleep(1. / 240.)

        ee_pos = get_ee_position()
        dist = np.linalg.norm(ee_pos - goal_pos)
        print(f"{iteration}] EE: {ee_pos.round(3)} | Distance: {dist:.4f}")
        iteration += 1

        p.removeState(state_id)

print(f"\nCSV written to: {output_csv_path}")