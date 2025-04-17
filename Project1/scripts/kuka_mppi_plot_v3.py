# ========= Description & Logs =================
# with GUI but storing all the end effector position (N, H, 3) in a CSV file.
# also storing the joint angles for each sequence in a separate CSV file.
# =======================================


from tracemalloc import start
import numpy as np
import pybullet as p
import pybullet_data
import time
import csv
import os
from datetime import datetime

# === Setup: PyBullet in GUI mode ===
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
num_joints = p.getNumJoints(robot_id)
ee_link_index = num_joints - 1
goal_pos = [0.4, 0.4, 0.2]

# === MPPI Parameters ===
N = 25
H = 10
sigma = 0.05
lambda_ = 1.0

output_dir = "/Users/rohith/MS/MR/Project1/data"  # ← Change if needed
os.makedirs(output_dir, exist_ok=True)
timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
output_csv_path = os.path.join(output_dir, f"mppi_ee_pos_{timestamp}.csv")
output_joint_csv_path = os.path.join(output_dir, f"mppi_joint_sequences_{timestamp}.csv")

# === Helper functions ===
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

# === Initialize ===
home = get_ee_position()
nominal_sequence = get_nominal_sequence(start_pos=home, goal_pos=goal_pos, H=H)
initial_joint_angles = get_joint_angles()
start_joint_angles = np.array([p.getJointState(robot_id, i)[0] for i in range(num_joints)])

# === Save EE Pos CSV Header ===
with open(output_csv_path, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(["iteration", "sequence_index", "timestep_index", "x", "y", "z"])
    
    # === Save Joint Sequence CSV Header ===
    with open(output_joint_csv_path, "w", newline="") as joint_csv:
        joint_writer = csv.writer(joint_csv)
        joint_writer.writerow(["iteration", "sequence_index", "timestep_index"] + [f"joint_{i}" for i in range(num_joints)])
        
        iteration = 1
        print(f"\n=== MPPI Iteration {iteration} ===")
        
        # Log initial EE position as sequence 0
        ee_pos = get_ee_position()
        writer.writerow([iteration, 0, 0, *ee_pos])

        # === Generate samples (N sequences with H steps each) ===
        noise = np.random.normal(0, sigma, size=(N, H, num_joints))
        samples = nominal_sequence[np.newaxis, :, :] + noise
        total_costs = np.zeros(N)

        # === Save nominal sequence (sequence_index = 0) ===
        for t in range(H):
            joint_writer.writerow([iteration, 0, t, *nominal_sequence[t]])

        # === Simulate all noisy sequences ===
        for i, sequence in enumerate(samples, start=1):
            for j in range(num_joints):
                p.resetJointState(robot_id, j, start_joint_angles[j])
            set_joint_angles(start_joint_angles)

            for _ in range(50):
                p.stepSimulation()

            for t in range(H):
                set_joint_angles(sequence[t])
                p.stepSimulation()
                time.sleep(1. / 240.)

                ee_pos = get_ee_position()
                collision_penalty = 100.0 if is_self_collision() or is_ground_collision() else 0.0
                total_costs[i - 1] += compute_cost(ee_pos, goal_pos, collision_penalty)
                writer.writerow([iteration, i, t, *ee_pos])
                joint_writer.writerow([iteration, i, t, *sequence[t]])

        print(f"Done. CSV written:\n  EE Pos:     {output_csv_path}\n  Joint Seq:  {output_joint_csv_path}")